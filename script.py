#!/usr/bin/env python3


"""
Allows to use the service dynamixel_command 
"""
from numpy import array
import rospy
import time
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

# keyboard
import termios, sys, os

TERMIOS = termios

def getkey(): #Captura la letra que ingresa del teclado
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    c = str(c).replace('b', "").replace('\'', "")
    return c






def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def deg2raw(input_list: list = [0,0,0,0,0], min_deg: int = -180, max_deg: int = 180)->list:
    out_list = [0,0,0,0,0]
    for i in range(len(input_list)):
        out_list[i] = int( ((input_list[i] - min_deg)*1024)/(max_deg-min_deg) )
    return out_list



def main(goal_position: list = [30,30,30,30,30], home_position: list = [0,0,0,0,0]):
    goal_position_raw = deg2raw(goal_position)
    home_position_raw = deg2raw(home_position)
    selected_link = "waist"
    print("--------------------------------")
    print("link: ",selected_link)
    
    # Goal_Position (0,1023)
    # Torque_Limit (0,1023)
    #jointCommand('', 1, 'Torque_Limit', 600, 0)
    #jointCommand('', 2, 'Torque_Limit', 400, 0)
    #jointCommand('', 3, 'Torque_Limit', 400, 0)
    #jointCommand('', 4, 'Torque_Limit', 400, 0)
    #jointCommand('', 5, 'Torque_Limit', 400, 0)

    while(True):
        key = getkey()
        
        if(selected_link == "waist"):
            if(key == "w"):
                selected_link = "shoulder"
                print("--------------------------------")
                print("link: ",selected_link)
            elif(key == "s"):
                print("--------------------------------")
                selected_link = "gripper"
                print("link: ",selected_link)
            elif(key == "d"):
                print("movement to goal ...")
                #jointCommand('', 1, 'Goal_Position', goal_position_raw(1), 0.5)
            elif(key == "a"):
                print("movement to home ...")
                #jointCommand('', 1, 'Goal_Position', home_position_raw(1), 0.5)
                
        elif(selected_link == "shoulder"):
            if(key == "w"):
                selected_link = "elbow"
                print("--------------------------------")
                print("link: ",selected_link)
            elif(key == "s"):
                print("--------------------------------")
                selected_link = "waist"
                print("link: ",selected_link)
            elif(key == "d"):
                print("movement to goal ...")
                #jointCommand('', 2, 'Goal_Position', goal_position_raw(2), 0.5)
            elif(key == "a"):
                print("movement to home ...")
                #jointCommand('', 2, 'Goal_Position', home_position_raw(2), 0.5)

        elif(selected_link == "elbow"):
            if(key == "w"):
                selected_link = "wrist"
                print("--------------------------------")
                print("link: ",selected_link)
            elif(key == "s"):
                print("--------------------------------")
                selected_link = "shoulder"
                print("link: ",selected_link)
            elif(key == "d"):
                print("movement to goal ...")
                #jointCommand('', 3, 'Goal_Position', goal_position_raw(2), 0.5)
            elif(key == "a"):
                print("movement to home ...")
                #jointCommand('', 3, 'Goal_Position', home_position_raw(2), 0.5)

        elif(selected_link == "wrist"):
            if(key == "w"):
                selected_link = "gripper"
                print("--------------------------------")
                print("link: ",selected_link)
            elif(key == "s"):
                print("--------------------------------")
                selected_link = "elbow"
                print("link: ",selected_link)
            elif(key == "d"):
                print("movement to goal ...")
                #jointCommand('', 4, 'Goal_Position', goal_position_raw(2), 0.5)
            elif(key == "a"):
                print("movement to home ...")
                #jointCommand('', 4, 'Goal_Position', home_position_raw(2), 0.5)


        elif(selected_link == "gripper"):
            if(key == "w"):
                selected_link = "waist"
                print("--------------------------------")
                print("link: ",selected_link)
            elif(key == "s"):
                print("--------------------------------")
                selected_link = "wrist"
                print("link: ",selected_link)
            elif(key == "d"):
                print("movement to goal ...")
                #jointCommand('', 5, 'Goal_Position', goal_position_raw(2), 0.5)
            elif(key == "a"):
                print("movement to home ...")
                #jointCommand('', 5, 'Goal_Position', home_position_raw(2), 0.5)

                



def main2():


    # Goal_Position (0,1023)
    # Torque_Limit (0,1023)
    jointCommand('', 1, 'Torque_Limit', 600, 0)
    jointCommand('', 2, 'Torque_Limit', 400, 0)
    jointCommand('', 3, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Goal_Position', 512, 0.5)
    jointCommand('', 3, 'Goal_Position', 512, 0.5)
    time.sleep(0.5)
    jointCommand('', 2, 'Goal_Position', 512, 0.5)
    time.sleep(0.5)
    jointCommand('', 1, 'Goal_Position', 512, 0.5)
    jointCommand('', 4, 'Goal_Position', 300, 1)
    time.sleep(0.2)
    jointCommand('', 2, 'Goal_Position', 600, 1)
    time.sleep(0.5)
    jointCommand('', 3, 'Goal_Position', 300, 0.5)
    jointCommand('', 1, 'Goal_Position', 512, 0.5)




if __name__ == '__main__':
    try:
        main()

        
    except rospy.ROSInterruptException:
        pass