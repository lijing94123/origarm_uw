#include "ros/ros.h"
#include "origarm_uw/Command_Pre_Open.h"
#include "origarm_uw/Command_ABL.h"

#include "myPID.h"
#include "PID_controller.cpp"
#include "myData.h"

float b1[SEGNUM];
float btem[SEGNUM];
float b2[SEGNUM];
float b3[SEGNUM];
float pressureD[SEGNUM][ACTNUM];
float alphad[SEGNUM];                //ABL desired value
float betad[SEGNUM];
float lengthd[SEGNUM];

float ajoy[SEGNUM];
float bjoy[SEGNUM];
float ljoy[SEGNUM];

float l0;

float Texta[SEGNUM];                 //external torque
float Textb[SEGNUM];
float Tx[SEGNUM];
float Ty[SEGNUM];
float Fl = 0;
float phycD[SEGNUM];
float physD[SEGNUM];
float phypD[SEGNUM]; 

float pLimitOptimal = 0;
float dalphaRatio[SEGNUM];
float dbeta[SEGNUM];
float dlength[SEGNUM];

float alphar[SEGNUM];                //ABL real value, calculated by sensor data
float betar[SEGNUM];
float lengthr[SEGNUM];

float phycFeedbackAlpha[SEGNUM];
float physFeedbackAlpha[SEGNUM];
float phycFeedbackBeta[SEGNUM];
float physFeedbackBeta[SEGNUM];
float phycFeedback[SEGNUM];
float physFeedback[SEGNUM];
float phypFeedback[SEGNUM];
float pressureDFeed[SEGNUM][ACTNUM];
float pressureDBack[SEGNUM][ACTNUM];

int feedbackFlag = 0;

float openingD[SEGNUM][ACTNUM];

using namespace std;

//initialization
PID_Type *alphaPID  = newPID(0.8, 0.005, 0, 0.005, 0.6, 1);
PID_Type *betaPID   = newPID(0.8, 0.005, 0, 0.005, 0.5, 1.5);
PID_Type *lengthPID = newPID(  1,  0.01, 0, 0.005, 0.1, 0.1);


void initParameter()
{
  for (int i = 0; i < SEGNUM; i++)
  {
    ajoy[i] = 0;
    bjoy[i] = 0;
    ljoy[i] = length0;
  }
}

void ABLD2PD_LMS()
{
    for (int i = 0; i < SEGNUM; i++)
    {
      Tx[i] = Texta[i]/crossA/radR-C1*alphad[i];
      Ty[i] = Textb[i]/crossA/radR;
      phycD[i] = cos(betad[i])*Tx[i]-sin(betad[i])*Ty[i];
      physD[i] = sin(betad[i])*Tx[i]+cos(betad[i])*Ty[i];
      
      l0 = length0;

      phypD[i] = (Fl+6*k0*(lengthd[i]-l0))/crossA;

      pressureDFeed[i][0] = phypD[i]/6+phycD[i]/3;
      pressureDFeed[i][1] = (phycD[i]+phypD[i]+1.7320508*physD[i])/6;
      pressureDFeed[i][2] = (phypD[i]-phycD[i]+1.7320508*physD[i])/6;
      pressureDFeed[i][3] = phypD[i]/6-phycD[i]/3;
      pressureDFeed[i][4] = (phypD[i]-phycD[i]-1.7320508*physD[i])/6;
      pressureDFeed[i][5] = (phypD[i]+phycD[i]-1.7320508*physD[i])/6;

      pressureDFeed[i][0]-=pLimitOptimal;
      pressureDFeed[i][1]+=pLimitOptimal;
      pressureDFeed[i][2]-=pLimitOptimal;
      pressureDFeed[i][3]+=pLimitOptimal;
      pressureDFeed[i][4]-=pLimitOptimal;
      pressureDFeed[i][5]+=pLimitOptimal;
    }
}

//calculate PressureFeedback
void FeedbackController(int feedbackFlag) 
{ 
    for (int i = 0; i < SEGNUM; i++)
    {
      alphad[i]  = ajoy[i];
      betad[i]   = bjoy[i];
      lengthd[i] = ljoy[i];
    }

  ABLD2PD_LMS();
    
  if(feedbackFlag)
  {
    for (int i = 0; i < SEGNUM; i++)
    {
      dalphaRatio[i] = updatePID(alphaPID,1,alphar[i]/alphad[i]);
      dbeta[i]       = updatePID(betaPID,betad[i],betar[i]);
      dlength[i]     = updatePID(lengthPID,lengthd[i],lengthr[i]);

      phycFeedbackBeta[i] = (cos(dbeta[i])-1)*phycD[i]-sin(dbeta[i])*physD[i];
      physFeedbackBeta[i] = sin(dbeta[i])*phycD[i]+(cos(dbeta[i])-1)*physD[i];

      phycFeedbackAlpha[i] = dalphaRatio[i]*(cos(dbeta[i])*phycD[i]-sin(dbeta[i])*physD[i]);
      physFeedbackAlpha[i] = dalphaRatio[i]*(sin(dbeta[i])*phycD[i]+cos(dbeta[i])*physD[i]);

      phycFeedback[i] = phycFeedbackBeta[i]+phycFeedbackAlpha[i];
      physFeedback[i] = physFeedbackBeta[i]+physFeedbackAlpha[i];
      phypFeedback[i] = 6*k0*dlength[i]/crossA;

      pressureDBack[i][0] = phypFeedback[i]/6+phycFeedback[i]/3;
      pressureDBack[i][1] = (phycFeedback[i]+phypFeedback[i]+1.7320508*physFeedback[i])/6;
      pressureDBack[i][2] = (phypFeedback[i]-phycFeedback[i]+1.7320508*physFeedback[i])/6;
      pressureDBack[i][3] = phypFeedback[i]/6-phycFeedback[i]/3;
      pressureDBack[i][4] = (phypFeedback[i]-phycFeedback[i]-1.7320508*physFeedback[i])/6;
      pressureDBack[i][5] = (phypFeedback[i]+phycFeedback[i]-1.7320508*physFeedback[i])/6;
    }  
  }
  else
  {
    for(int i = 0; i < SEGNUM; i++)
    {
      for (int j = 0; j < ACTNUM; j++)
      {
        pressureDBack[i][j] = 0;
      }
    }
  }

  for (int i = 0; i < SEGNUM; i++)
  {
    for (int j = 0; j < ACTNUM; j++)
    {
        pressureD[i][j] = pressureDFeed[i][j] + pressureDBack[i][j];
    }
  }
}

class ABL_controller
{
  public:
    ABL_controller()
    {
      sub1_ = n_.subscribe("Cmd_ABL_joy", 1, &ABL_controller::ABL_joy, this);
      pub_ = n_.advertise<origarm_uw::Command_Pre_Open>("Command_Pre_Open", 100);
    }

    void ABL_joy(const origarm_uw::Command_ABL& msg)
    {

      for (int i = 0; i < SEGNUM; i++)
      {
        ajoy[i] = msg.segment[i].A;
        bjoy[i] = msg.segment[i].B;
        ljoy[i] = msg.segment[i].L;    
      }
      ROS_INFO("GET ABL Command from JOY!");
  }
   
    void pub()
    {
      for (int i = 0; i < SEGNUM; i++)
      {
        for (int j = 0; j < ACTNUM; j++)
        {
          Cmd_P_O.segment[i].command[j].pressure = pressureD[i][j]/100;     //send hPa to spi_node 
          Cmd_P_O.segment[i].command[j].valve    = 1;                       //bool == 1, commandType == pressureCommandType
        }
      }

      pub_.publish(Cmd_P_O);
    }

    private:
      ros::NodeHandle n_;
      ros::Subscriber sub1_;
      ros::Subscriber sub2_;
      ros::Publisher pub_ ;

      origarm_uw::Command_Pre_Open Cmd_P_O;
      origarm_uw::Command_ABL Command_ABL;

  };

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ABL_controller_node");

  initParameter();

  ABL_controller ABL_controller_node;

  ros::AsyncSpinner s(5);
  s.start();

  ros::Rate loop_rate(100); 
	
  ROS_INFO("Ready for ABL_controller_node");

  while(ros::ok())
  {
    //ABLD2PD();
    FeedbackController(feedbackFlag);

    ABL_controller_node.pub();

    /*for (int i = 0; i < seg; i++)
    {
      printf("Command[%d]: %f %f %f %f %f %f\r\n", i, 
        pressureD[i][0], 
        pressureD[i][1],
        pressureD[i][2], 
        pressureD[i][3],
        pressureD[i][4],
        pressureD[i][5]);     
    }*/

    //ros::spinOnce();
    
    loop_rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
