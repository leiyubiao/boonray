#include "lidar_localization/path_tracking/purePursuit.hpp"
namespace lidar_localization
{
	purePursuit::purePursuit(const vector<Position> &wayPoints, float wheelBase, float setIndex,const purePursuit::PID_variables& setPID)
	{
		m_wheelBase = wheelBase;
		m_setIndex = setIndex;
		m_lowerIndex = 0;
		m_flagForFirst = true;

        m_pid = setPID;
        // setPID: Kp_brake, ki_brake, kd_brake must be negative.
        // v: m/s
        // Kp_v = 1.5, Ki_v = 0.0, kd_v = 0.5;
        // Kp_brake = -8.0, Ki_v = 0.0, Kd_brake = -2.0;
        // purePursuit::PID_variables setPID;

        /*setPID.Kp_v = 1.5;
        setPID.Ki_v = 0.0;
        setPID.Kd_v = 0.5;
        setPID.Kp_brake = -8.0;
        setPID.Ki_brake = -0.0;
        setPID.Kd_brake = -2.0;*/

		m_error = 0.0;
		m_error_pre = 0.0;
		m_error_sum = 0.0;



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

void purePursuit::PID_forSpeed(const float expected_speed, const float actual_speed) // reserved;
{

    m_error_pre = m_error;
    m_error = expected_speed - actual_speed;
    m_error_sum += m_error;
	cout<<"m_error = "<<m_error<<endl;
    if (m_error < -1.5/2.0f) // 5.4 km/h
    {
        m_cmd.speed = 0.0f;
        m_cmd.brake = m_pid.Kp_brake*m_error +  m_pid.Ki_brake*m_error_sum + m_pid.Kd_brake*(m_error-m_error_pre);
		//cout<<"brake kp="<< m_pid.Kp_brake<<"brake ki = "<<m_pid.Ki_brake<<"brake kd="<<m_pid.Kd_brake<<endl;


    }else{
        m_cmd.brake = 0.0f;
		cout<<"actual_speed = "<<  actual_speed<<endl;
        m_cmd.speed = actual_speed + m_pid.Kp_v * m_error + 0*m_pid.Ki_v*m_error_sum + m_pid.Kd_v*(m_error-m_error_pre);
		//cout<<"kp = "<<m_pid.Kp_v<<"ki ="<< m_pid.Ki_v <<"kd = "<<m_pid.Kd_v<<endl;
		//cout<<"m_error = "<<m_error<<"error_pre="<<m_error_pre<<"error_sum"<<m_error_sum<<endl;
		//cout<<"m_cmd.speed="<< m_cmd.speed<<endl;
    }

    m_cmd.speed = min(5.0f,max(0.0f,m_cmd.speed));
	cout<<"  speed =  "<< m_cmd.speed<<endl;
	cout<<" brake = "<<m_cmd.brake<<endl;
    m_cmd.brake = min(60.0f,max(0.0f,m_cmd.brake));


}
purePursuit::ctrlCommand purePursuit::VehicleControl(const Position &vehiclePos, const int aeb,  const float expected_speed,  const float actual_speed)
	{
		//ctrlCommand cmd;
		float expected_v = expected_speed;
		float actual_v = actual_speed;

        //expected_v = 3.0f;
		//if (m_nearestIndex > 40 && m_nearestIndex<70) expected_v = 5.0f;
     //   if (m_nearestIndex > 70 && m_nearestIndex<m_upperIndex) expected_v = 2.0f;
		 if (aeb == 1 ||abs(m_upperIndex - m_nearestIndex) < 25)
        {
          expected_v = 0.0f;
        }
        PID_forSpeed( expected_v,  actual_v);



		FindIndex(vehiclePos);
		Tranfer(vehiclePos);
		float steerCmd;
		float ld;
		float yValue;
		ld = sqrt(m_vehiclePos.x * m_vehiclePos.x + m_vehiclePos.y * m_vehiclePos.y);
		
		yValue = -m_vehiclePos.y;
		steerCmd = atan(2 * m_wheelBase * yValue / (ld * ld)) * 180.0 / 3.1415926; // if it is wrong, please add "-" before atan();
		steerCmd = min(25.0f, max(-25.0f,steerCmd));
		m_cmd.steer = steerCmd;
		cout<<"the calcuted x = "<< m_vehiclePos.x <<" , "<<m_vehiclePos.y<<endl;
		cout<<"steer="<<steerCmd<<"  speed ="<< m_cmd.speed<<"  brake = "<<m_cmd.brake<<endl;// deg
        return m_cmd;
	}

} // namespace lidar_localization



