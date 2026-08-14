#!/usr/bin/env python3
"""
fis2c.py — Generate portable C99 code from a MATLAB Fuzzy Inference System (.fis) file.

Usage:
    python3 fis2c.py fuzzy_type1.fis [-o OUTDIR] [-N 201]

Emits <Name>.h, <Name>.c, and <Name>_demo.c (Name taken from [System] Name=).
The generated module evaluates the FIS using the methods declared in the .fis
(AND / implication / aggregation / defuzzification). Edit the .fis and re-run to
update the C — no hand editing of the C is ever needed.

Supported:
  - Mamdani (centroid / mom defuzz) and Sugeno with 'constant' output MFs.
  - MF types: gaussmf, zmf, smf, trimf, trapmf, gbellmf, pimf, sigmf, constant.
  - AND/implication: min | prod ; aggregation: max | probor.
  - Any number of inputs / outputs, don't-care (0) inputs, rule weights.
Pure Python stdlib, no dependencies.
"""

import argparse
import os
import re
import sys

# MF type -> (enum tag, expected parameter count)
MF_TABLE = {
    'gaussmf':  ('MF_GAUSSMF', 2),
    'zmf':      ('MF_ZMF', 2),
    'smf':      ('MF_SMF', 2),
    'trimf':    ('MF_TRIMF', 3),
    'trapmf':   ('MF_TRAPMF', 4),
    'gbellmf':  ('MF_GBELL', 3),
    'pimf':     ('MF_PIMF', 4),
    'sigmf':    ('MF_SIGMF', 2),
    'constant': ('MF_CONST', 1),
}


# --------------------------------------------------------------------------- #
# FIS data model
# --------------------------------------------------------------------------- #
class MF:
    def __init__(self, name, mtype, params):
        self.name = name
        if mtype not in MF_TABLE:
            raise ValueError(
                "Unsupported MF type '%s' for MF '%s'. Supported: %s"
                % (mtype, name, sorted(MF_TABLE)))
        self.mtype = mtype
        expected = MF_TABLE[mtype][1]
        if len(params) != expected:
            raise ValueError(
                "MF '%s' (%s) expects %d params, got %d: %s"
                % (name, mtype, expected, len(params), params))
        self.params = params


class Var:
    def __init__(self, name, rng, mfs):
        self.name = name
        self.range = rng
        self.mfs = mfs


class Rule:
    def __init__(self, in_idx, out_idx, weight, conn):
        self.in_idx = in_idx
        self.out_idx = out_idx
        self.weight = weight
        self.conn = conn  # 1 = AND, 2 = OR


class FIS:
    def __init__(self):
        self.name = 'fuzzy'
        self.ftype = 'mamdani'
        self.and_m = 'min'
        self.or_m = 'max'
        self.imp_m = 'min'
        self.agg_m = 'max'
        self.defuzz = 'centroid'
        self.inputs = []
        self.outputs = []
        self.rules = []


# --------------------------------------------------------------------------- #
# Parser
# --------------------------------------------------------------------------- #
def _stripq(s):
    return s.strip().strip("'").strip('"')


def parse_fis(path):
    fis = FIS()
    sections = {}
    cur = None
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line[0] in '#%':
                continue
            m = re.match(r'^\[(.+)\]\s*$', line)
            if m:
                cur = m.group(1).strip()
                sections.setdefault(cur, [])
            elif cur is not None:
                sections[cur].append(line)

    # [System]
    for ln in sections.get('System', []):
        if '=' not in ln:
            continue
        k, v = ln.split('=', 1)
        k = k.strip()
        v = v.strip()
        if k == 'Name':
            fis.name = _stripq(v)
        elif k == 'Type':
            fis.ftype = _stripq(v).lower()
        elif k == 'AndMethod':
            fis.and_m = _stripq(v).lower()
        elif k == 'OrMethod':
            fis.or_m = _stripq(v).lower()
        elif k == 'ImpMethod':
            fis.imp_m = _stripq(v).lower()
        elif k == 'AggMethod':
            fis.agg_m = _stripq(v).lower()
        elif k == 'DefuzzMethod':
            fis.defuzz = _stripq(v).lower()

    def parse_var_block(lines):
        name = None
        rng = None
        mfs = []
        for ln in lines:
            if ln.startswith('Name='):
                name = _stripq(ln.split('=', 1)[1])
            elif ln.startswith('Range='):
                inside = ln[ln.index('[') + 1: ln.index(']')]
                rng = [float(x) for x in re.split(r'[,\s]+', inside.strip()) if x]
            elif re.match(r"MF\d+\s*=", ln):
                mm = re.match(
                    r"MF\d+\s*=\s*'([^']*)'\s*:\s*'([^']*)'\s*,\s*\[([^\]]*)\]", ln)
                if not mm:
                    raise ValueError("Cannot parse MF line: %s" % ln)
                pname = mm.group(1)
                ptype = mm.group(2).lower()
                params = [float(x) for x in re.split(r'[,\s]+', mm.group(3).strip()) if x]
                mfs.append(MF(pname, ptype, params))
        if rng is None or len(rng) != 2:
            raise ValueError("Variable block missing a valid Range=[lo hi]: %r" % (rng,))
        return Var(name, rng, mfs)

    in_keys = sorted([k for k in sections if re.match(r'^Input\d+$', k)],
                     key=lambda s: int(s[5:]))
    out_keys = sorted([k for k in sections if re.match(r'^Output\d+$', k)],
                      key=lambda s: int(s[6:]))
    fis.inputs = [parse_var_block(sections[k]) for k in in_keys]
    fis.outputs = [parse_var_block(sections[k]) for k in out_keys]

    # [Rules]  format: i j ... , k l ... (w) : conn
    for ln in sections.get('Rules', []):
        if ln[0] in '#%':
            continue
        conn = 1
        if ':' in ln:
            main, conn_s = ln.split(':', 1)
            try:
                conn = int(conn_s.strip())
            except ValueError:
                conn = 1
        else:
            main = ln
        weight = 1.0
        wm = re.search(r'\(([^)]*)\)', main)
        if wm:
            try:
                weight = float(wm.group(1))
            except ValueError:
                weight = 1.0
            main = main[:wm.start()] + main[wm.end():]
        if ',' not in main:
            raise ValueError("Bad rule (missing input/output separator ','): %s" % ln)
        left, right = main.split(',', 1)
        in_idx = [int(x) for x in left.split()]
        out_idx = [int(x) for x in right.split()]
        fis.rules.append(Rule(in_idx, out_idx, weight, conn))

    return fis


# --------------------------------------------------------------------------- #
# C emission helpers
# --------------------------------------------------------------------------- #
def cident(s):
    s = re.sub(r'[^0-9A-Za-z_]', '_', s or 'fuzzy')
    if s and s[0].isdigit():
        s = '_' + s
    return s or 'fuzzy'


def flit(x):
    """C float literal from a Python float (lossless repr + f suffix)."""
    return repr(float(x)) + 'f'


def mf_literal(mf):
    tag = MF_TABLE[mf.mtype][0]
    p = list(mf.params) + [0.0] * (4 - len(mf.params))
    ps = ', '.join(flit(v) for v in p)
    return '{%s, {%s}}' % (tag, ps)


# Generic C prolog: MF type enum, MFDef struct, helper MF math, and mf_eval.
# Identical for every FIS.
C_PROLOG = r'''typedef enum {
    MF_GAUSSMF, MF_ZMF, MF_SMF, MF_TRIMF, MF_TRAPMF, MF_GBELL, MF_PIMF, MF_SIGMF, MF_CONST
} MFType;

typedef struct { MFType type; float p[4]; } MFDef;

static float mu_gauss(float s, float c, float x) {
    if (s == 0.0f) return (x == c) ? 1.0f : 0.0f;
    float d = (x - c) / s;
    return expf(-0.5f * d * d);
}
static float mu_zmf(float a, float b, float x) {
    if (a == b) return (x <= a) ? 1.0f : 0.0f;
    if (x <= a) return 1.0f;
    if (x >= b) return 0.0f;
    float m = 0.5f * (a + b), t;
    if (x <= m) { t = (x - a) / (b - a); return 1.0f - 2.0f * t * t; }
    t = (b - x) / (b - a);
    return 2.0f * t * t;
}
static float mu_smf(float a, float b, float x) {
    if (a == b) return (x >= a) ? 1.0f : 0.0f;
    if (x <= a) return 0.0f;
    if (x >= b) return 1.0f;
    float m = 0.5f * (a + b), t;
    if (x <= m) { t = (x - a) / (b - a); return 2.0f * t * t; }
    t = (b - x) / (b - a);
    return 1.0f - 2.0f * t * t;
}
static float mf_eval(MFType type, const float p[4], float x) {
    switch (type) {
    case MF_GAUSSMF: return mu_gauss(p[0], p[1], x);
    case MF_ZMF:     return mu_zmf(p[0], p[1], x);
    case MF_SMF:     return mu_smf(p[0], p[1], x);
    case MF_TRIMF: {
        float a = p[0], b = p[1], c = p[2];
        if (x <= a || x >= c) return 0.0f;
        if (x <= b) { if (b == a) return 1.0f; return (x - a) / (b - a); }
        if (c == b) return 1.0f;
        return (c - x) / (c - b);
    }
    case MF_TRAPMF: {
        float a = p[0], b = p[1], c = p[2], d = p[3];
        if (x <= a || x >= d) return 0.0f;
        if (x >= b && x <= c) return 1.0f;
        if (x < b) { if (b == a) return 1.0f; return (x - a) / (b - a); }
        if (d == c) return 1.0f;
        return (d - x) / (d - c);
    }
    case MF_GBELL: {
        float a = p[0], b = p[1], c = p[2];
        if (a == 0.0f) return (x == c) ? 1.0f : 0.0f;
        float t = fabsf((x - c) / a);
        return 1.0f / (1.0f + powf(t, 2.0f * b));
    }
    case MF_PIMF: {
        float s = mu_smf(p[0], p[1], x), z = mu_zmf(p[2], p[3], x);
        return (s < z) ? s : z;
    }
    case MF_SIGMF:
        return 1.0f / (1.0f + expf(-p[0] * (x - p[1])));
    case MF_CONST:
        return p[0];
    }
    return 0.0f;
}
'''


def tnorm_func(method):
    if method in ('min', 'minimum'):
        return 'static float t_norm(float a, float b) { return (a < b) ? a : b; } /* AND=min */'
    if method in ('prod', 'product'):
        return 'static float t_norm(float a, float b) { return a * b; } /* AND=prod */'
    sys.stderr.write("warning: AndMethod '%s' unsupported, defaulting to min\n" % method)
    return 'static float t_norm(float a, float b) { return (a < b) ? a : b; } /* AND=min (default) */'


def snorm_func(method):
    if method in ('max', 'maximum'):
        return 'static float s_norm(float a, float b) { return (a > b) ? a : b; } /* AGG=max */'
    if method in ('probor', 'probsum', 'algebraic'):
        return 'static float s_norm(float a, float b) { return a + b - a * b; } /* AGG=probor */'
    sys.stderr.write("warning: AggMethod '%s' unsupported, defaulting to max\n" % method)
    return 'static float s_norm(float a, float b) { return (a > b) ? a : b; } /* AGG=max (default) */'


def imp_func(method):
    if method in ('min', 'minimum'):
        return 'static float imp(float w, float mu) { return (w < mu) ? w : mu; } /* IMP=min */'
    if method in ('prod', 'product'):
        return 'static float imp(float w, float mu) { return w * mu; } /* IMP=prod */'
    sys.stderr.write("warning: ImpMethod '%s' unsupported, defaulting to min\n" % method)
    return 'static float imp(float w, float mu) { return (w < mu) ? w : mu; } /* IMP=min (default) */'


# --------------------------------------------------------------------------- #
# C emission
# --------------------------------------------------------------------------- #
def _fmt_range(rng):
    return '[%.6g, %.6g]' % (rng[0], rng[1])


def emit_header(fis, prefix):
    guard = prefix.upper() + '_H'
    ni = len(fis.inputs)
    no = len(fis.outputs)
    args = ', '.join('float %s' % cident(v.name) for v in fis.inputs)
    argnames = ', '.join(cident(v.name) for v in fis.inputs)
    in_lines = '\n'.join(
        ' *   [%d] %-10s range %s' % (i, "'%s'" % (v.name or '?'), _fmt_range(v.range))
        for i, v in enumerate(fis.inputs))
    out_lines = '\n'.join(
        ' *   [%d] %-10s range %s' % (i, "'%s'" % (v.name or '?'), _fmt_range(v.range))
        for i, v in enumerate(fis.outputs))

    L = []
    # ---- file-level documentation block (data-driven from the .fis) ----
    L.append('/*')
    L.append(' * %s.h -- Fuzzy inference evaluator (AUTO-GENERATED).' % prefix)
    L.append(' *')
    L.append(' * Generated by fis2c.py from FIS "%s". DO NOT EDIT BY HAND.' % fis.name)
    L.append(' * To change rules / membership functions, edit the .fis and re-run:')
    L.append(' *     python3 fis2c.py %s.fis' % fis.name)
    L.append(' *')
    L.append(' * System    : %s fuzzy inference' % fis.ftype)
    L.append(' * Inputs    : %d   (pass them to the evaluator in the order below)' % ni)
    L.append(' * Outputs   : %d' % no)
    L.append(' * Methods   : AND=%s  implication=%s  aggregation=%s  defuzz=%s'
             % (fis.and_m, fis.imp_m, fis.agg_m, fis.defuzz))
    L.append(' *')
    L.append(' * Inputs (evaluator argument order):')
    L.append(in_lines)
    L.append(' *')
    L.append(' * Outputs:')
    L.append(out_lines)
    L.append(' *')
    L.append(' * Notes:')
    L.append(' *   - Inputs are NOT clamped internally; the membership functions are')
    L.append(' *     defined over the whole real line, so out-of-range values still')
    L.append(' *     evaluate. Scale your physical signals into the ranges above')
    L.append(' *     (e.g. normalized error in [-1, 1]) before calling.')
    L.append(' *   - The evaluator is reentrant (no mutable static state) and safe to')
    L.append(' *     call from multiple threads / a real-time control loop.')
    L.append(' *   - Cost per call is O(N_rules * N_output_samples); trivial for a')
    L.append(' *     kHz-class control loop.')
    L.append(' *')
    L.append(' * Usage (C):')
    L.append(' *     #include "%s.h"' % prefix)
    if no == 1:
        L.append(' *     float u = %s_eval(%s);   // single-output shortcut' % (prefix, argnames))
    L.append(' *     // -- or, array form --')
    L.append(' *     float in[%d] = {%s};' % (ni, argnames))
    L.append(' *     float out[%d];' % no)
    L.append(' *     %s_eval_core(in, out);' % prefix)
    L.append(' *')
    L.append(' * Usage (C++): the declarations below are wrapped in extern "C",')
    L.append(' *     so just #include this header directly from a .cpp file.')
    L.append(' *')
    L.append(' * Build:')
    L.append(' *     gcc  app.c %s.c -lm' % prefix)
    L.append(' *     g++  app.cpp %s.c -lm' % prefix)
    L.append(' */')
    L.append('')
    L.append('#ifndef %s' % guard)
    L.append('#define %s' % guard)
    L.append('')
    L.append('#ifdef __cplusplus')
    L.append('extern "C" {')
    L.append('#endif')
    L.append('')
    # ---- core evaluator ----
    L.append('/*')
    L.append(' * %s_eval_core -- evaluate the FIS from flat arrays.' % prefix)
    L.append(' *')
    L.append(' *   in[%d] : input values, in the order listed at the top of this file' % ni)
    L.append(' *           (%s).' % argnames)
    L.append(' *   out[%d]: resulting output value(s).' % no)
    L.append(' *')
    L.append(' * Use this form when the number of inputs/outputs is not known at')
    L.append(' * compile time, or from a generic controller loop. Reentrant.')
    L.append(' */')
    L.append('void %s_eval_core(const float * in, float * out);' % prefix)
    L.append('')
    # ---- convenience wrapper ----
    if no == 1:
        L.append('/*')
        L.append(' * %s_eval -- convenience wrapper for the single-output case.' % prefix)
        L.append(' *')
        for v in fis.inputs:
            L.append(' *   %-12s : input %s' % (cident(v.name), "'%s'" % (v.name or '?')))
        L.append(' *   returns    : out[0] (%s).' % ("'%s'" % (fis.outputs[0].name or '?')))
        L.append(' *')
        L.append(' * Equivalent to packing the arguments into in[%d] and returning out[0].' % ni)
        L.append(' */')
        L.append('float %s_eval(%s);' % (prefix, args))
        L.append('')
    L.append('#ifdef __cplusplus')
    L.append('}')
    L.append('#endif')
    L.append('')
    L.append('#endif /* %s */' % guard)
    return '\n'.join(L) + '\n'


def emit_source(fis, prefix, N):
    ni = len(fis.inputs)
    no = len(fis.outputs)
    nr = len(fis.rules)
    maxmi = max((len(v.mfs) for v in fis.inputs), default=0)
    maxmo = max((len(v.mfs) for v in fis.outputs), default=0)

    if fis.ftype not in ('mamdani', 'sugeno', 'sugeno-type'):
        sys.stderr.write("warning: Type '%s' unrecognized; assuming mamdani.\n" % fis.ftype)
        fis.ftype = 'mamdani'
    is_sugeno = fis.ftype.startswith('sugeno')

    L = []
    L.append('/* Auto-generated by fis2c.py from FIS "%s". Do not edit by hand. */' % fis.name)
    L.append('#include "%s.h"' % prefix)
    L.append('#include <math.h>')
    L.append('')
    L.append('#define FUZZY_NI %d' % ni)
    L.append('#define FUZZY_NO %d' % no)
    L.append('#define FUZZY_NR %d' % nr)
    L.append('#define FUZZY_MAXMI %d' % maxmi)
    L.append('#define FUZZY_MAXMO %d' % maxmo)
    L.append('#define FUZZY_N %d' % N)
    L.append('')
    L.append(C_PROLOG.rstrip())
    L.append('')
    L.append(tnorm_func(fis.and_m))
    L.append(snorm_func(fis.agg_m))
    if not is_sugeno:
        L.append(imp_func(fis.imp_m))
    L.append('')

    # Input MF table  (pad each input to MAXMI with MF_CONST{0})
    pad = '{MF_CONST, {0.0f, 0.0f, 0.0f, 0.0f}}'
    L.append('static const int in_nmf[FUZZY_NI] = {%s};'
             % ', '.join(str(len(v.mfs)) for v in fis.inputs))
    rows = []
    for v in fis.inputs:
        ents = [mf_literal(m) for m in v.mfs] + [pad] * (maxmi - len(v.mfs))
        rows.append('  {' + ', '.join(ents) + '}')
    L.append('static const MFDef in_mf[FUZZY_NI][FUZZY_MAXMI] = {')
    L.append(',\n'.join(rows))
    L.append('};')
    L.append('')

    # Output MF table
    L.append('static const int out_nmf[FUZZY_NO] = {%s};'
             % ', '.join(str(len(v.mfs)) for v in fis.outputs))
    rows = []
    for v in fis.outputs:
        ents = [mf_literal(m) for m in v.mfs] + [pad] * (maxmo - len(v.mfs))
        rows.append('  {' + ', '.join(ents) + '}')
    L.append('static const MFDef out_mf[FUZZY_NO][FUZZY_MAXMO] = {')
    L.append(',\n'.join(rows))
    L.append('};')
    L.append('')

    # Output ranges
    L.append('static const float out_lo[FUZZY_NO] = {%s};'
             % ', '.join(flit(v.range[0]) for v in fis.outputs))
    L.append('static const float out_hi[FUZZY_NO] = {%s};'
             % ', '.join(flit(v.range[1]) for v in fis.outputs))
    L.append('')

    # Rule table
    def rule_lit(r):
        # pad in_idx to NI, out_idx to NO (don't-care = 0)
        ins = list(r.in_idx) + [0] * (ni - len(r.in_idx))
        outs = list(r.out_idx) + [0] * (no - len(r.out_idx))
        if len(r.in_idx) > ni:
            raise ValueError("rule has %d input indices but FIS has %d inputs: %s"
                             % (len(r.in_idx), ni, r.in_idx))
        if len(r.out_idx) > no:
            raise ValueError("rule has %d output indices but FIS has %d outputs: %s"
                             % (len(r.out_idx), no, r.out_idx))
        return '{ { %s }, { %s }, %s, %d }' % (
            ', '.join(str(x) for x in ins),
            ', '.join(str(x) for x in outs),
            flit(r.weight), r.conn)

    L.append('typedef struct { int in[FUZZY_NI]; int out[FUZZY_NO]; float weight; int conn; } FuzzyRule;')
    L.append('static const FuzzyRule rules[FUZZY_NR] = {')
    for i, r in enumerate(fis.rules):
        comma = ',' if i < nr - 1 else ''
        L.append('  %s%s' % (rule_lit(r), comma))
    L.append('};')
    L.append('')

    # Firing strengths are precomputed once per call (independent of output sample).
    # Then aggregate over the output universe.
    if is_sugeno:
        L.append(emit_sugeno_eval_core(prefix))
    else:
        L.append(emit_mamdani_eval_core(prefix, fis.defuzz))

    L.append('')
    if no == 1:
        argnames = ', '.join(cident(v.name) for v in fis.inputs)
        L.append('float %s_eval(%s) {' % (prefix, ', '.join('float %s' % cident(v.name) for v in fis.inputs)))
        L.append('    float in[FUZZY_NI] = {%s};' % argnames)
        L.append('    float out[FUZZY_NO];')
        L.append('    %s_eval_core(in, out);' % prefix)
        L.append('    return out[0];')
        L.append('}')
    return '\n'.join(L) + '\n'


def emit_mamdani_eval_core(prefix, defuzz):
    """Centroid (or mom) defuzz over a discretized output universe."""
    if defuzz in ('mom', 'lom', 'som', 'middle'):
        finalize = (
            '            {\n'
            '                float mx = 0.0f;\n'
            '                for (int k = 0; k < FUZZY_N; k++) if (agg[k] > mx) mx = agg[k];\n'
            '                float num = 0.0f, den = 0.0f;\n'
            '                for (int k = 0; k < FUZZY_N; k++) {\n'
            '                    if (agg[k] >= mx - 1e-6f) {\n'
            '                        float y = out_lo[o] + (out_hi[o] - out_lo[o]) * '
            '(FUZZY_N == 1 ? 0.0f : (float) k / (float) (FUZZY_N - 1));\n'
            '                        num += y; den += 1.0f;\n'
            '                    }\n'
            '                }\n'
            '                out[o] = (den > 0.0f) ? num / den : 0.0f;\n'
            '            }\n')
    else:
        if defuzz != 'centroid':
            sys.stderr.write(
                "warning: DefuzzMethod '%s' unsupported, using centroid.\n" % defuzz)
        finalize = (
            '            {\n'
            '                float num = 0.0f, den = 0.0f;\n'
            '                for (int k = 0; k < FUZZY_N; k++) {\n'
            '                    float y = out_lo[o] + (out_hi[o] - out_lo[o]) * '
            '(FUZZY_N == 1 ? 0.0f : (float) k / (float) (FUZZY_N - 1));\n'
            '                    num += y * agg[k];\n'
            '                    den += agg[k];\n'
            '                }\n'
            '                out[o] = (den > 1e-9f) ? num / den : 0.0f;\n'
            '            }\n')

    return (
        'void %s_eval_core(const float * in, float * out) {\n'
        '    /* Input memberships. */\n'
        '    float mu_in[FUZZY_NI][FUZZY_MAXMI];\n'
        '    for (int i = 0; i < FUZZY_NI; i++)\n'
        '        for (int m = 0; m < in_nmf[i]; m++)\n'
        '            mu_in[i][m] = mf_eval(in_mf[i][m].type, in_mf[i][m].p, in[i]);\n'
        '\n'
        '    /* Firing strength per rule (combine inputs by AND=t_norm / OR=s_norm). */\n'
        '    float firing[FUZZY_NR];\n'
        '    for (int r = 0; r < FUZZY_NR; r++) {\n'
        '        float f = 1.0f;\n'
        '        int seen = 0;\n'
        '        for (int ii = 0; ii < FUZZY_NI; ii++) {\n'
        '            int idx = rules[r].in[ii];\n'
        '            if (idx != 0) {\n'
        '                float v = mu_in[ii][idx - 1];\n'
        '                f = seen ? ((rules[r].conn == 2) ? s_norm(f, v) : t_norm(f, v)) : v;\n'
        '                seen = 1;\n'
        '            }\n'
        '        }\n'
        '        firing[r] = (seen ? f : 1.0f) * rules[r].weight;\n'
        '    }\n'
        '\n'
        '    /* Precompute each output MF over the discretized universe. */\n'
        '    float mu_out[FUZZY_NO][FUZZY_MAXMO][FUZZY_N];\n'
        '    for (int o = 0; o < FUZZY_NO; o++)\n'
        '        for (int m = 0; m < out_nmf[o]; m++)\n'
        '            for (int k = 0; k < FUZZY_N; k++) {\n'
        '                float y = out_lo[o] + (out_hi[o] - out_lo[o]) * '
        '(FUZZY_N == 1 ? 0.0f : (float) k / (float) (FUZZY_N - 1));\n'
        '                mu_out[o][m][k] = mf_eval(out_mf[o][m].type, out_mf[o][m].p, y);\n'
        '            }\n'
        '\n'
        '    /* Aggregate + defuzzify each output. */\n'
        '    for (int o = 0; o < FUZZY_NO; o++) {\n'
        '        float agg[FUZZY_N];\n'
        '        for (int k = 0; k < FUZZY_N; k++) {\n'
        '            float a = 0.0f;\n'
        '            for (int r = 0; r < FUZZY_NR; r++) {\n'
        '                int oi = rules[r].out[o];\n'
        '                if (oi == 0) continue;\n'
        '                a = s_norm(a, imp(firing[r], mu_out[o][oi - 1][k]));\n'
        '            }\n'
        '            agg[k] = a;\n'
        '        }\n'
        '%s'
        '    }\n'
        '}\n' % (prefix, finalize))


def emit_sugeno_eval_core(prefix):
    """Sugeno with constant output MFs: weighted average of singletons."""
    return (
        'void %s_eval_core(const float * in, float * out) {\n'
        '    /* Input memberships. */\n'
        '    float mu_in[FUZZY_NI][FUZZY_MAXMI];\n'
        '    for (int i = 0; i < FUZZY_NI; i++)\n'
        '        for (int m = 0; m < in_nmf[i]; m++)\n'
        '            mu_in[i][m] = mf_eval(in_mf[i][m].type, in_mf[i][m].p, in[i]);\n'
        '\n'
        '    /* Firing strength per rule. */\n'
        '    float firing[FUZZY_NR];\n'
        '    for (int r = 0; r < FUZZY_NR; r++) {\n'
        '        float f = 1.0f;\n'
        '        int seen = 0;\n'
        '        for (int ii = 0; ii < FUZZY_NI; ii++) {\n'
        '            int idx = rules[r].in[ii];\n'
        '            if (idx != 0) {\n'
        '                float v = mu_in[ii][idx - 1];\n'
        '                f = seen ? ((rules[r].conn == 2) ? s_norm(f, v) : t_norm(f, v)) : v;\n'
        '                seen = 1;\n'
        '            }\n'
        '        }\n'
        '        firing[r] = (seen ? f : 1.0f) * rules[r].weight;\n'
        '    }\n'
        '\n'
        '    /* Weighted average of constant output singletons. */\n'
        '    for (int o = 0; o < FUZZY_NO; o++) {\n'
        '        float num = 0.0f, den = 0.0f;\n'
        '        for (int r = 0; r < FUZZY_NR; r++) {\n'
        '            int oi = rules[r].out[o];\n'
        '            if (oi == 0) continue;\n'
        '            float cval = out_mf[o][oi - 1].p[0];\n'
        '            num += firing[r] * cval;\n'
        '            den += firing[r];\n'
        '        }\n'
        '        out[o] = (den > 1e-9f) ? num / den : 0.0f;\n'
        '    }\n'
        '}\n' % prefix)


def emit_demo(fis, prefix):
    ni = len(fis.inputs)
    no = len(fis.outputs)
    names = [cident(v.name) for v in fis.inputs]
    return (
        '/* Auto-generated demo for FIS "%s". Reads %d input(s) from argv, prints %d output(s). */\n'
        '#include "%s.h"\n'
        '#include <stdio.h>\n'
        '#include <stdlib.h>\n'
        '\n'
        'int main(int argc, char ** argv) {\n'
        '    if (argc != %d + 1) {\n'
        '        fprintf(stderr, "usage: %%s %s\\n", argv[0]);\n'
        '        return 1;\n'
        '    }\n'
        '    float in[%d];\n'
        '    for (int i = 0; i < %d; i++) in[i] = (float) atof(argv[i + 1]);\n'
        '    float out[%d];\n'
        '    %s_eval_core(in, out);\n'
        '    for (int i = 0; i < %d; i++)\n'
        '        printf("%%g%%c", out[i], (i + 1 < %d) ? \' \' : \'\\n\');\n'
        '    return 0;\n'
        '}\n' % (fis.name, ni, no, prefix, ni, ' '.join(names), ni, ni, no, prefix, no, no))


# --------------------------------------------------------------------------- #
# Summary / verification printout
# --------------------------------------------------------------------------- #
def print_summary(fis):
    w = sys.stderr
    print("FIS '%s' (%s): %d input(s), %d output(s), %d rule(s)"
          % (fis.name, fis.ftype, len(fis.inputs), len(fis.outputs), len(fis.rules)), file=w)
    for kind, vars_ in (('Input', fis.inputs), ('Output', fis.outputs)):
        for v in vars_:
            print("  %s '%s' Range=[%.4g %.4g] %d MFs"
                  % (kind, v.name, v.range[0], v.range[1], len(v.mfs)), file=w)
            for m in v.mfs:
                print("      %s : %s %s" % (m.name, m.mtype, m.params), file=w)
    print("  Methods: AND=%s OR=%s IMP=%s AGG=%s DEFUZZ=%s"
          % (fis.and_m, fis.or_m, fis.imp_m, fis.agg_m, fis.defuzz), file=w)
    print("  Rules:", file=w)
    for r in fis.rules:
        print("      in=%s out=%s w=%g conn=%d"
              % (r.in_idx, r.out_idx, r.weight, r.conn), file=w)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate C99 from a MATLAB .fis file.")
    ap.add_argument('fis', help="path to the .fis file")
    ap.add_argument('-o', '--outdir', default=None,
                    help="output directory (default: same dir as the .fis)")
    ap.add_argument('-N', type=int, default=201,
                    help="output discretization points for centroid (default: 201)")
    args = ap.parse_args(argv)

    fis = parse_fis(args.fis)
    print_summary(fis)

    prefix = cident(fis.name)
    outdir = args.outdir or os.path.dirname(os.path.abspath(args.fis))
    os.makedirs(outdir, exist_ok=True)

    h_path = os.path.join(outdir, prefix + '.h')
    c_path = os.path.join(outdir, prefix + '.c')
    demo_path = os.path.join(outdir, prefix + '_demo.c')

    with open(h_path, 'w') as f:
        f.write(emit_header(fis, prefix))
    with open(c_path, 'w') as f:
        f.write(emit_source(fis, prefix, args.N))
    with open(demo_path, 'w') as f:
        f.write(emit_demo(fis, prefix))

    print("generated: %s, %s, %s" % (h_path, c_path, demo_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
