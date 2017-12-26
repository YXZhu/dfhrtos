#include "pid.h"
#include<stdio.h>
#include<stdlib.h>

//struct{
//    float SetSpeed;            //�����趨ֵ
//    float ActualSpeed;        //����ʵ��ֵ
//    float err;                //����ƫ��ֵ
//    float err_next;            //������һ��ƫ��ֵ
//    float err_last;            //��������ǰ��ƫ��ֵ
//    float Kp,Ki,Kd;            //������������֡�΢��ϵ��
//}pid;

void PID_init(){
	pid pid;
    pid.SetSpeed=0.0;
    pid.ActualSpeed=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.err_next=0.0;
    pid.Kp=0.2;
    pid.Ki=0.015;
    pid.Kd=0.2;
}

float PID_realize(float speed){
	 pid pid;
    pid.SetSpeed=speed;
    pid.err=pid.SetSpeed-pid.ActualSpeed;
    float incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
    pid.ActualSpeed+=incrementSpeed;
    pid.err_last=pid.err_next;
    pid.err_next=pid.err;
    return pid.ActualSpeed;
}