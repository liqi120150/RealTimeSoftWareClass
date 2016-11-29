#include <stdio.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>


// Data Type

typedef struct
{
    bool Request;
    bool Response;
    bool Done;
    double Position;
    double Velocity;
    double Acceleration;
    double Deceleration;
    double Jerk;
} tPosCmd;

typedef struct
{
    double Position;
    double Velocity;
} tAxisSetpoint;

//Global variables

RT_TASK task_trajectory_generator;
RT_TASK task_command_sender;

tPosCmd new_cmd;
tAxisSetpoint axis1_setpoint;

int cycle_count = 0;

/*****************************************************
 * status machine
 * 
*****************************************************/
typedef enum
{
	Idel = 0,
	Accelerate,
	Decelerate,
	UniformVelocity	
}trajectoryStatusDef;

#define Position_DeadSize	10
#define Velocity_DeadSize	10
void task_trajectory_generator_proc(void *arg)
{
	RTIME now, previous;
	trajectoryStatusDef Status = Idel;

	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 0.1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 100000000);
	previous = rt_timer_read();

	axis1_setpoint.Position = 0;
	axis1_setpoint.Velocity = 0;

	int temp;

	float delta_T;
	while (1) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		/*
		 * NOTE: printf may have unexpected impact on the timing of
		 *       your program. It is used here in the critical loop
		 *       only for demonstration purposes.
		 */
		/*
		printf("Task A Time since last turn: %ld.%06ld ms\n",
		       (long)(now - previous) / 1000000,
		       (long)(now - previous) % 1000000);
		*/
		       

		//  Add your code
		delta_T = ((double)(now - previous) / 1000000000);
		previous = now;
		
		if(new_cmd.Request)
		{
			printf("Position:%10.3f,Velocity:%10.3f at time:%10.3f s.\r\n", 
				axis1_setpoint.Position,
				axis1_setpoint.Velocity,
				(double)now / 1000000000);
			if(fabs(new_cmd.Position - axis1_setpoint.Position) > Position_DeadSize)
			{
				if(fabs(new_cmd.Velocity - axis1_setpoint.Velocity) > Velocity_DeadSize)
				{
					if(fabs((axis1_setpoint.Velocity * axis1_setpoint.Velocity) / (2*new_cmd.Deceleration)) >= fabs(new_cmd.Position - axis1_setpoint.Position))
					{
						if(axis1_setpoint.Velocity > 0)
							Status = Decelerate;
						else
							Status = Accelerate;
					}
					else if(new_cmd.Velocity > axis1_setpoint.Velocity)
						Status = Accelerate;
					else
						Status = Decelerate;
				}
				else
				{
					if(fabs((axis1_setpoint.Velocity * axis1_setpoint.Velocity) / (2*new_cmd.Deceleration)) >= fabs(new_cmd.Position - axis1_setpoint.Position))
					{
						if(axis1_setpoint.Velocity > 0)
							Status = Decelerate;
						else
							Status = Accelerate;
					}
					else
						Status = UniformVelocity;
				}
			}
			else
			{
				Status = Idel;
				new_cmd.Response = true;
				new_cmd.Done = true;
			}
			switch(Status)
			{
				case Accelerate:
					//s = s0 + v0*delta_T + 0.5*acce*(delta_T^2)
					axis1_setpoint.Position += axis1_setpoint.Velocity*delta_T + 0.5*new_cmd.Acceleration * (delta_T * delta_T);
					//v = v0 + acce*delta_T
					axis1_setpoint.Velocity += new_cmd.Acceleration*delta_T;
					break;
				case Decelerate:
					//s = s0 + v0*delta_T - 0.5*Deceleration*(delta_T^2)
					axis1_setpoint.Position += axis1_setpoint.Velocity*delta_T - 0.5*new_cmd.Deceleration * (delta_T * delta_T);
					//v = v0 - Deceleration*delta_T
					axis1_setpoint.Velocity -= new_cmd.Deceleration*delta_T;	
					break;
				case UniformVelocity:
					//s = s0 + v0*delta_T;
					axis1_setpoint.Position += axis1_setpoint.Velocity*delta_T;
					break;
				case Idel:
					break;
				default:
					break;
			}
		}
			   
		
	}
}

void task_command_sender_proc(void *arg)
{
	RTIME now, previous;

	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 2 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 2000000000);
	previous = rt_timer_read();

        new_cmd.Request = false;
        new_cmd.Response = false;
        new_cmd.Done = false;
        new_cmd.Position = 0;
        new_cmd.Velocity = 0;
        new_cmd.Acceleration = 0;
        new_cmd.Deceleration = 0;
        new_cmd.Jerk = 0;

	while (1) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		/*
		 * NOTE: printf may have unexpected impact on the timing of
		 *       your program. It is used here in the critical loop
		 *       only for demonstration purposes.
		 */
		 /*
		printf("Task B Time since last turn: %ld.%06ld ms\n",
		       (long)(now - previous) / 1000000,
		       (long)(now - previous) % 1000000);
		*/
		       previous = now;
                cycle_count = cycle_count + 1;
                printf("cycle_count:%d\n",cycle_count);
        
                if(cycle_count == 5)
                {
                    new_cmd.Request = true;
                    new_cmd.Response = false;
                    new_cmd.Done = false;
                    new_cmd.Position = 20000;
                    new_cmd.Velocity = 1000;
                    new_cmd.Acceleration = 50;
                    new_cmd.Deceleration = 50;
                    new_cmd.Jerk = 0;

                }

	}
}

void catch_signal(int sig)
{

}


int main(int argc, char* argv[])
{
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

        printf("A simple motion control demo\n");
	/*
	 * Arguments: &task,
	 *            name,
	 *            stack size (0=default),
	 *            priority,
	 *            mode (FPU, start suspended, ...)
	 */
	rt_task_create(&task_trajectory_generator, "rttask_trajectory_generator", 0, 99, 0);
	rt_task_create(&task_command_sender, "rttask_command_sender", 0, 98, 0);

	/*
	 * Arguments: &task,
	 *            task function,
	 *            function argument
	 */
	rt_task_start(&task_trajectory_generator, &task_trajectory_generator_proc, NULL);
	rt_task_start(&task_command_sender, &task_command_sender_proc, NULL);

        while(!new_cmd.Done);
        printf("End! \n");
	rt_task_delete(&task_trajectory_generator);
	rt_task_delete(&task_command_sender);
	return 0;
}
