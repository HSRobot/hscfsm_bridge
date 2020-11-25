#! /bin/bash
a=("pose1" "pose2")
isContinue='y'
robot0=(2 4 5 6 7 8 9 10 17)
rboto1=(0 1 3 11 12 13 14 15 16)


for((i=1; i<18; i++)); do
    read -n1 -p 'continue?[y/n/c]' isContinue
    if [ "${isContinue}" == "n" ] || [ "${isContinue}" == "N" ]; then
        echo "break"
        break
    elif [ "${isContinue}" == "c" ] || [ "${isContinue}" == "C" ]; then
        echo "continue"
        continue
    else
        echo "'1, ${i}'"

rosservice call /VoiceCtlRob_TaskServerCmd "type: 1
taskName: 'record'
behavior: 'next'
param:
- '1'
- '${i}'" 

    while :
    do
        read -n1 -p '按下記錄動作?[y]' isContinue
        if [ ${isContinue} == "y" ]; then
            break
        fi
    done

    if [[ ${robot0[*]} =~ $i ]]; then
rosservice call /record_pose "robot: 0
uri: 'pose'
PoseName: '${a[$i]}'
isJointSpace: false"
    else
rosservice call /record_pose "robot: 1
uri: 'pose'
PoseName: '${a[$i]}'
isJointSpace: false"
    fi
done



