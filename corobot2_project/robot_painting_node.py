# 미완성 로봇 드로잉 코드

import time
import rclpy
import DR_init
from onrobot import RG

# 그리퍼 관련 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
# 그리퍼 객체 생성
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
SYSTEM_COOKING = 0
SYSTEM_DONE = 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("robot_painting_node", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            movec,
            moveb,
            movesx,
            move_periodic,
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            check_position_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            set_singular_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            wait,
            DR_BASE,
            DR_AVOID,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_MVS_VEL_CONST,
            DR_TOOL,
            DR_LINE,
        )

        from DR_common2 import posj, posx, posb

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    home = posj(0, 0, 90, 0, 90, 0) # 홈 위치

    brush_pos = posx(448.65, 227.59, 165.33, 178.57, -136.2, 91.45)
    water_pos = posx(538.0, 139.0, 200., 466.7, -176.7, 76.9)
    tissue_pos = posx(538.0, 70.0, 300., 466.7, -176.7, 76.9)
    paint_pos_1 = posx(538.0, 0.0, 280.7, 466.7, -176.7, 76.9)
    paint_pos_2 = posx(538.0, -70.0, 280.7, 466.7, -176.7, 76.9)
    paint_zero = posx(338.0, 227.0, 195, 466.7, -176.7, 76.9)
    
    set_singular_handling(DR_AVOID)
    set_velj(30.0)
    set_accj(30.0)
    set_velx(100.0, 40.625)
    set_accx(500.0, 161.5)

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def trans(pos: list[float], delta: list[float], ref: int = DR_BASE, ref_out: int = DR_BASE) -> list[float]:

        if len(pos) != 6 or len(delta) != 6:
            raise ValueError("pos와 delta는 각각 6개의 float 값을 포함하는 리스트여야 합니다.")

        # 결과를 저장할 새로운 리스트 생성
        result_pos = []

        # 각 인덱스에 맞춰 값 더하기
        for i in range(6):
            result_pos.append(pos[i] + delta[i])

        return result_pos

    def force_ctrl():
        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[1500, 1500, 1500, 200, 200, 200]) #힘 제어 시작
        time.sleep(0.5)
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        print("Starting set_desired_force")
        set_desired_force(fd=fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 

        # 외력이 0 이상 5 이하이면 0
        # 외력이 5 초과이면 -1
        while not check_force_condition(DR_AXIS_Z, max=5):
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass

        print("Starting release_force")
        release_force()
        time.sleep(0.5)

        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()

        return True

    import numpy as np
    def sine_list(points1, points2):
        # points1 = [277.145, 7.384, 34.081, 109.384, 179.97, 109.228]
        # points2 = [707.81, 7.384, 34.081, 109.384, 179.97, 109.228]

        period = points2[0] - points1[0]
        # period = points2[1] - points1[1]
        x = np.arange(0, 2 * np.pi, 0.1)
        y = np.sin(x)

        x *= period / (2 * np.pi)
        y *= period / (2 * np.pi)

        x += points1[0]
        y += points1[1]
        sine_list = []
        for i in range(len(x)):
            pos = [x[i], y[i]] + points1[2:] 
            sine_list.append(posx(pos))
        return sine_list

    def sine_motion(points1, points2, num_points=100, vel=[100, 30], acc=[100, 60]):
        """
        두 점(points1, points2)을 기준으로 사인 곡선을 따라 이동 궤적을 생성하고 실행.
        points1, points2 = [x, y, z, rx, ry, rz] 형태
        """

        # --- X축 구간 길이 (주기) ---
        period = points2[0] - points1[0]

        # --- Y축 차이로 진폭 계산 ---
        amplitude = (points2[1] - points1[1]) / 2.0

        # --- 사인파 생성 ---
        x = np.linspace(0, 2 * np.pi, num_points)
        y = np.sin(x)

        # --- 크기 보정 ---
        x = x * (period / (2 * np.pi)) + points1[0]   # X축 위치 맞추기
        y = y * amplitude + (points1[1] + points2[1]) / 2.0  # Y축 중앙 기준

        # --- trajectory 리스트 만들기 ---
        sine_list = []
        for i in range(len(x)):
            pos = [x[i], y[i]] + points1[2:]  # z, rx, ry, rz는 points1 값 유지
            sine_list.append(posx(pos))       # posx는 로봇 좌표 변환 함수라고 가정

        # --- 로봇 이동 ---
        movesx(sine_list, vel=vel, acc=acc, ref=DR_BASE, vel_opt=DR_MVS_VEL_CONST)

        return sine_list

######################################################################3
    def delta_x(x):
        return [x, 0, 0, 0, 0, 0]
    def delta_y(y):
        return [0, y, 0, 0, 0, 0]
    def delta_z(z):
        return [0, 0, z, 0, 0, 0]
    
    def drawing_z_setting():
        cp = get_current_posx()[0]
        print(cp[2])
        return cp[2]
    
    def pick_brush(brush_pos, del_z = 10.0):
        print("pick_brush")
        gripper.move_gripper(300)
        movel(trans(brush_pos, delta_z(del_z)))
        wait(0.3)
        movel(brush_pos)
        gripper.close_gripper()
        wait(0.3)
        movel([0, 0, +134, 0, 0, 0], mod=DR_MV_MOD_REL) #올려!!!!!!!!
        wait(0.3)
        # movel([538.0, 236.0, 300.7, 466.7, -176.7, 76.9])
    def put_brush(brush_pos, del_z = 10.0):
        print("put_brush")
        movej(home)
        movel(trans(brush_pos, delta_z(del_z)))
        wait(0.3)
        movel(brush_pos)
        gripper.move_gripper(300)
        wait(0.3)
        movel([0, 0, +134, 0, 0, 0], mod=DR_MV_MOD_REL) #올려!!!!!!!!
        wait(0.3)
        # movel([538.0, 236.0, 300.7, 466.7, -176.7, 76.9])

    def brush_go_water(water_pos, del_z = 10.0):
        print("brush_go_water")
        movel(trans(water_pos, delta_z(del_z)))
        wait(0.3)
        movel(water_pos)
        wait(0.3)

        watering()
        movel([0, 0, +80, 0, 0, 0], mod=DR_MV_MOD_REL, vel=30) #올려!!!!!!!!
        print(get_current_posx()[0][2])
        wait(0.3)


    def watering(): # 물 흔들기
        print('watering')
        amp = [0, 0, 0, 3, 3, 0]
        period = [0, 0, 0, 1, 1, 0]
        repeat = 2
        atime = 0.2
        ref = DR_TOOL
        move_periodic(amp, period, atime, repeat, ref)

    def brush_rub(paint_pos):
        print('brush_rub')
        paint_pos[2] = 280
        movel(paint_pos) 
        movel([0, 0, -20, 0, 0, 0], mod=DR_MV_MOD_REL) #살짝 내림

        wait(0.5)
        
        amp = [0, 0, 0, 0, 0, 10]
        period = [0, 0, 0, 0, 0, 2]
        repeat = 2
        atime = 0.2
        ref = DR_TOOL
        if force_ctrl():
            movel([0, 0, 3, 0, 0, 0], mod=DR_MV_MOD_REL) #살짝 올림
            move_periodic(amp, period, atime, repeat, ref)  # 물감 바르기
        wait(0.5)
        cur_p = get_current_posx()[0]
        cur_p[2] = 280
        
        movel(cur_p)

    def drawing_sin(pos1, pos2):
        print('drawing_sin')

        # cur_pos = get_current_posx()[0]
        # cur_pos[2] = pos1[2]
        # s_list = sine_list(cur_pos, pos1)
        # movesx(s_list, vel=[100, 30], acc=[100, 60], ref=DR_BASE, vel_opt=DR_MVS_VEL_CONST)
        # pos1[2] = drawing_z_setting()[2]
        movel(pos1)
        # if force_ctrl():
        if True:
            # set_z = drawing_z_setting()
            # pos1[2] = set_z
            # pos2[2] = set_z
            
            s_list = sine_list(pos1, pos2)
            movesx(s_list, vel=[100, 30], acc=[100, 30], ref=DR_BASE, vel_opt=DR_MVS_VEL_CONST)


    
    def drawing_R(start_pose):
        p1 = trans(start_pose, delta_x(-40))
        p2 = trans(p1, delta_y(20))
        p3 = trans(p2, delta_x(20))
        p4 = trans(p3, delta_y(-20))
        p5 = trans(p4, delta_y(10))
        p  = trans(p5, delta_x(20))
        p6 = trans(p , delta_y(10))
        
        r_points = [start_pose, p1, p2, p3, p4, p5, p6]
        
        pos_list = []
        movel(trans(start_pose, delta_z(15)))
        for point in r_points:
            movel(point)
            # pos_list.append(posb(DR_LINE, point, radius=1))
        # moveb(pos_list, vel=20, acc=20)

    def drawing_O(start_pose):
        p1 = trans(start_pose, delta_x(-40))
        p2 = trans(p1, delta_y(20))
        p3 = trans(p2, delta_x(40))
        p4 = trans(p3, delta_y(-20))
        
        r_points = [start_pose, p1, p2, p3, p4]
        
        pos_list = []
        movel(trans(start_pose, delta_z(15)))
        for point in r_points:
            # movel(point)
            pos_list.append(posb(DR_LINE, point, radius=1))
        moveb(pos_list, vel=50, acc=40)

    def drawing_K(start_pose):
        p1 = trans(start_pose, delta_x(-40))
        p2 = trans(p1, delta_x(20))
        p  = trans(p2, delta_x(-20))
        p3 = trans(p , delta_y(20))
        p4 = p2
        p  = trans(p2, delta_x(20))
        p5 = trans(p , delta_y(20))
        r_points = [start_pose, p1, p2, p3, p4, p5]
        
        pos_list = []
        movel(trans(start_pose, delta_z(15)))
        for point in r_points:
            movel(point)
        #     pos_list.append(posb(DR_LINE, point, radius=1))
        # moveb(pos_list, vel=20, acc=20)

    def drawing_E(start_pose):
        p1 = trans(start_pose, delta_y(20))
        p2 = trans(p1, delta_y(-20))
        p3 = trans(p2, delta_x(-40))
        p4 = trans(p3, delta_y(20))
        p5 = trans(p4, delta_y(-20))
        p6 = trans(p5, delta_x(20))
        p7 = trans(p6, delta_y(20))

        r_points = [start_pose, p1, p2, p3, p4, p5, p6, p7]
        
        pos_list = []
        movel(trans(start_pose, delta_z(15)))
        for point in r_points:
            movel(point)
        #     pos_list.append(posb(DR_LINE, point, radius=1))
        # moveb(pos_list, vel=20, acc=20)

    def drawing_Y(start_pose):
        p1 = trans(start_pose, delta_y(10))
        p2 = trans(p1, delta_x(-30))
        p  = trans(p2, delta_x(-15))
        p3 = trans(p , delta_y(15))
        p4 = p2
        p_  = trans(p4, delta_x(-15))
        p5 = trans(p_ , delta_y(-15))
        r_points = [start_pose, p1, p2, p3, p4, p5]
        
        pos_list = []
        movel(trans(start_pose, delta_z(15)))
        for point in r_points:
            movel(point)
        #     pos_list.append(posb(DR_LINE, point, radius=1))
        # moveb(pos_list, vel=20, acc=20)


    while rclpy.ok():
        print(f"Moving to joint position: {home}")
        movej(home, vel=40, acc=20)  # 홈 위치로 이동

        pick_brush(brush_pos, 50.0)
        brush_go_water(water_pos, 80.0)

        brush_rub(tissue_pos)
        brush_rub(paint_pos_2)
        z_velue = [327.145, -207.384, 200.081, 109.384, 179.97, 109.228]

        movel(z_velue)
        if force_ctrl():
            movel([0, 0, 5, 0, 0, 0], mod=DR_MV_MOD_REL) #살짝 올림
        cp = get_current_posx()[0]

        sp = [277.145, -207.384, 200.081, 109.384, 179.97, 109.228]
        movel(sp)

        sp[2] = cp[2]
        sp1 = trans(cp , delta_y(30))
        sp2 = trans(sp1, delta_y(30))
        sp3 = trans(sp2, delta_y(30))
        sp4 = trans(sp3, delta_y(30))
        sp5 = trans(sp4, delta_y(30))

        drawing_R(sp1)
        brush_rub(paint_pos_2)
        drawing_O(sp2)
        brush_rub(paint_pos_2)
        drawing_K(sp3)
        brush_rub(paint_pos_2)
        drawing_E(sp4)
        brush_rub(paint_pos_2)
        drawing_Y(sp5)

        put_brush(brush_pos, 100)
        
        # p1 = [277.145, -207.384, 34.081, 109.384, 179.97, 109.228]
        # p2 = [277.145, 307.384, 34.081, 109.384, 179.97, 109.228]
        # drawing_sin(p1, p2)
        
        # points1 = [277.145, 7.384, 34.081, 109.384, 179.97, 109.228]
        # points2 = [707.81, 7.384, 34.081, 109.384, 179.97, 109.228]
        # drawing_sin(points1, points2)

        # sine_motion(p1, p2, num_points=100, vel=[100, 30], acc=[100, 60])
        
        movej(home)
        wait(2.0)

        break

        

    rclpy.shutdown()


if __name__ == "__main__":
    main()



