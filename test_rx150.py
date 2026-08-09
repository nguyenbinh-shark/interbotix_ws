from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

def main():
    # Khởi tạo kết nối với rx150
    bot = InterbotixManipulatorXS(
        robot_model='rx150',
        group_name='arm',
        gripper_name='gripper'
    )

    print("Đang đưa robot về vị trí Home...")
    bot.arm.go_to_home_pose()
    time.sleep(2)

    print("Đang mở kẹp...")
    bot.gripper.release()
    time.sleep(1)

    print("Đang đóng kẹp...")
    bot.gripper.grasp()
    time.sleep(1)

    print("Đưa robot về vị trí nghỉ (Sleep)...")
    bot.arm.go_to_sleep_pose()
    print("Hoàn thành!")

if __name__ == '__main__':
    main()