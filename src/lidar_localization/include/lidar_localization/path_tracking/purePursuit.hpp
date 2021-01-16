#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include<ros/ros.h>
#include<geometry_msgs/Point32.h>
using namespace std;



namespace lidar_localization
{
	class purePursuit
	{

	public:
		struct Position
		{

			float x;
			float y;
			float heading;
			Position(float _x = 0.0f, float _y = 0.0f, float _heading = 0.0f) : x(_x), y(_y), heading(_heading) {}
			Position &operator=(const Position &_tmp)
			{
				x = _tmp.x;
				y = _tmp.y;
				heading = _tmp.heading;
				return *this;
			}
		};

		struct ctrlCommand
		{

			float steer; // deg
			float speed;
			float brake;
			ctrlCommand(float _steer=0.0f, float _speed=0.0f, float _brake=0.0f): steer(_steer),speed(_speed),brake(_brake){}
			ctrlCommand &operator=(const ctrlCommand &ctrl)
			{
				steer = ctrl.steer;
				speed = ctrl.speed;
				brake = ctrl.brake;
				return *this;
			}
		};

		struct PID_variables
        {
		    float Kp_v;
		    float Ki_v;
		    float Kd_v;

		    float Kp_brake;
		    float Ki_brake;
		    float Kd_brake;

            /*PID_variables(float _kp_v, float _ki_v, float _kd_v, float _kp_brake, float _ki_brake, float _kd_brake){
                Kp_v = _kp_v;
                Ki_v = _ki_v;
                Kd_v = _kd_v;
                Kp_brake = _kp_brake;
                Ki_brake = _ki_brake;
                Kd_brake = _kd_brake;
            };*/
		    PID_variables &operator = (const PID_variables& _pid)
            {
		        Kp_v = _pid.Kp_v;
		        Ki_v = _pid.Ki_v;
		        Kd_v = _pid.Kd_v;

		        Kp_brake = _pid.Kp_brake;
		        Ki_v = _pid.Kp_brake;
		        Kd_v = _pid.Kd_brake;
		        return *this;
            }

        };
	public:
		Position m_globalPos; // 全局坐标系下的预瞄点。
		ctrlCommand m_cmd;
		PID_variables m_pid;
		int m_foundIndex; // 预瞄点的索引。
		int m_nearestIndex; // 最近点的索引。

	private:
		vector<Position> m_wayPoints; //set the wappoints as global parameter
		bool m_flagForFirst;
		int m_lowerIndex;
		int m_upperIndex;
		int m_setIndex;


		float m_wheelBase;

		float m_speed;
		float m_brake;

		
		Position m_vehiclePos;
	private: // variables for PID controller
	    float m_error;
		float m_error_pre;
		float m_error_sum;



	private:
		
		void Tranfer(const Position &vehiclePos);
		void PID_forSpeed(const float expected_speed, const float actual_speed); // reserved;

	public:
		purePursuit(const vector<Position> &wayPoints, float wheelBase, float setIndex,const PID_variables& setPID);
		void FindIndex(const Position &vehiclePos);
        ctrlCommand VehicleControl(const Position &vehiclePos, const int aeb,  const float expected_speed,  const float actual_speed);
        //ctrlCommand GetCmd();
		~purePursuit(){};
	};
}
#endif

/*****************************************************************************************/
	/*purePursuit::purePursuit(const vector<purePursuit::Position> &wayPoints, float wheelBase, float setIndex)
	{
		m_wheelBase = wheelBase;
		m_setIndex = setIndex;
		m_lowerIndex = 0;
		m_flagForFirst = true;
		m_upperIndex = wayPoints.size();
		if (m_lowerIndex >= m_upperIndex)
		{
			cout << "Warning: no data!!!!" << endl;
		}

		for (int i = 0; i < m_upperIndex; i++)
		{

			m_wayPoints.push_back(wayPoints[i]);
		}

		m_foundIndex = 0;
		m_brake = 0.0;
		m_speed = 0.0;
		m_nearestIndex = 0;
	}

	void purePursuit::FindIndex(const purePursuit::Position &vehiclePos)
	{

		float minValue = 999999999999999999.0;
		float dx;
		float dy;
		float dist;

		if (m_flagForFirst)
		{

			for (int i = 0; i < m_upperIndex; i++)
			{
				dx = vehiclePos.x - m_wayPoints[i].x;
				dy = vehiclePos.y - m_wayPoints[i].y;
				dist = sqrt(dx * dx + dy * dy);
				if (dist < minValue)
				{

					minValue = dist;
					m_nearestIndex = i;
				}
			}

			m_flagForFirst = false;
		}
		else
		{
			int searchRange = 20;
			int upper, lower;
			lower = max(0, m_nearestIndex - searchRange);
			upper = min(m_upperIndex - 1, m_nearestIndex + searchRange);
			for (int i = lower; i < upper; i++)
			{
				dx = vehiclePos.x - m_wayPoints[i].x;
				dy = vehiclePos.y - m_wayPoints[i].y;
				dist = sqrt(dx * dx + dy * dy);
				if (dist < minValue)
				{

					minValue = dist;
					m_nearestIndex = i;
				}
			}
		}

		m_foundIndex = m_nearestIndex + m_setIndex;
		m_foundIndex = min(m_foundIndex,m_upperIndex-1);
	}

	void purePursuit::Tranfer(const purePursuit::Position &vehiclePos)
	{

		m_globalPos = m_wayPoints[m_foundIndex];

		float theta = vehiclePos.heading * 3.1415926 / 180.0;
		float tmp_x, tmp_y;
		tmp_x = m_globalPos.x - vehiclePos.x;
		tmp_y = m_globalPos.y - vehiclePos.y;
		m_vehiclePos.x = cos(theta) * tmp_x - sin(theta) * tmp_y;
		m_vehiclePos.y = sin(theta) * tmp_x + cos(theta) * tmp_y;
	}

	purePursuit::ctrlCommand purePursuit::VehicleControl(const purePursuit::Position &vehiclePos, const int aeb )
	{
		ctrlCommand cmd;
		if (aeb == 1 ||abs(m_upperIndex - m_nearestIndex) < 5)
		{
			cmd.speed = 0.0f;
		}
		else if (aeb == 0)
		{
			cmd.speed = 3.3f;
			//PIDforSpeed();
		}
		FindIndex(vehiclePos);
		Tranfer(vehiclePos);
		float steerCmd;
		float ld;
		float yValue;
		ld = sqrt(m_vehiclePos.x * m_vehiclePos.x + m_vehiclePos.y * m_vehiclePos.y);
		
		yValue = -m_vehiclePos.y;
		steerCmd = atan(2 * m_wheelBase * yValue / (ld * ld)) * 180.0 / 3.1415926; // if it is wrong, please add "-" before atan();
		steerCmd = min(25.0f, max(-25.0f,steerCmd));
		cmd.steer = steerCmd;
		cout<<"the calcuted x = "<< m_vehiclePos.x <<" , "<<m_vehiclePos.y<<endl;
		cout<<"steer="<<steerCmd<<endl;
		return cmd;
	}
} // namespace lidar_localization*/
//#endif
