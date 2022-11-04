int wheelbase=30, dt=1;
const int SIZE=200;
double vl=10, vr=8;
double rob_x[SIZE]={30}, rob_y[SIZE]={30}, rob_theta[SIZE]={PI/4};
// To generate the robot trajectory using robot kinematic equations
int robot_kinematics(double left_vel, double right_vel)
{
int i;
for(i=0; i<SIZE; i++)
{
rob_x[i+1]=rob_x[i]+(vl+vr)/2*cos(rob_theta[i])dt;
rob_y[i+1]=rob_y[i]+(vl+vr)/2sin(rob_theta[i])dt;
rob_theta[i+1]=rob_theta[i]+(vr-vl)/wheelbasedt;
// Put your code for implementing kinematic equations here
}
return 0;
}
// You should write main function into the file.
FILE *fp; // data file for saving the trajectory data
int main(int argc, char **argv)
{
int i;
fp = fopen ("trajectory1","w");
robot_kinematics(vl, vr);
fprintf(fp, “%d, %d \n”, 30, 30);
for(i=1; i<SIZE; i++){
fprintf(fp, “%f, %f \n”, rob_x[i], rob_y[i]);
}
fclose(fp);
return 0;
}
