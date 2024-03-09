#include<windows.h>
#include<math.h>
#include<fstream>
#include<ctime>
#include<iostream>
#include <sstream>
#include<string>
#include <vector>
#include <HRSDK.h>

#include "SixAxis.h"
#include "ReadAndWriteFile.h"

using namespace std;

HROBOT device_id;

int CountLines(char* filename)
{
	ifstream ReadFile;
	int n = 0;
	string tmp;
	ReadFile.open(filename, ios::in);//ios::in 表示以只讀的方式讀取檔案

	while (getline(ReadFile, tmp, '\n'))
	{
		n++;
	}
	ReadFile.close();
	return n;

}

vector<vector<double>> read_jointvalue(string csv_path)
{
	std::ifstream ImportTrajectory(csv_path);

	if (!ImportTrajectory)
		std::cout << "open " << csv_path << " fail." << std::endl << std::endl;
	else
		std::cout << "open " << csv_path << " success." << std::endl << std::endl;

	std::vector<double>jointValue0;
	std::vector<double>jointValue1;
	std::vector<double>jointValue2;
	std::vector<double>jointValue3;
	std::vector<double>jointValue4;
	std::vector<double>jointValue5;
	std::string Data;

	double value;
	int count = 0;

	while (1)
	{
		ImportTrajectory >> Data;

		if (Data != "end")
		{
			int joint = count % 6;
			value = std::stof(Data);

			if (joint == 0)
				jointValue0.push_back(value);
			else if (joint == 1)
				jointValue1.push_back(value);
			else if (joint == 2)
				jointValue2.push_back(value);
			else if (joint == 3)
				jointValue3.push_back(value);
			else if (joint == 4)
				jointValue4.push_back(value);
			else if (joint == 5)
				jointValue5.push_back(value);

		}
		else
			break;
		count++;
	}

	if (jointValue0.size() == jointValue5.size())
		std::cout << "& The trajectory amount inputed: " << ((jointValue0.size() - 90) / 10) + 3 << " Layer, " << jointValue0.size() << " points" << std::endl;

	vector<vector<double>> jointValue{ jointValue0, jointValue1, jointValue2, jointValue3, jointValue4, jointValue5 };

	return jointValue;
}

// HIWIN robot function
void __stdcall callBack(uint16_t, uint16_t, uint16_t*, int)
{

}

int main()
{	
	// 凸點分別讀檔
	//jointValue = read_jointvalue(fileName);
	std::string fileName = "path0.csv";
	std::ifstream ImportTrajectory(fileName);

	if (!ImportTrajectory)
		std::cout << "open " << fileName << " fail." << std::endl << std::endl;
	else
		std::cout << "open " << fileName << " success." << std::endl << std::endl;

	std::vector<double>jointValue0;
	std::vector<double>jointValue1;
	std::vector<double>jointValue2;
	std::vector<double>jointValue3;
	std::vector<double>jointValue4;
	std::vector<double>jointValue5;
	std::string Data;

	double value;
	int count = 0;

	while (1)
	{
		ImportTrajectory >> Data;

		if (Data != "end")
		{
			int joint = count % 6;
			value = std::stof(Data);

			if (joint == 0)
				jointValue0.push_back(value);
			else if (joint == 1)
				jointValue1.push_back(value);
			else if (joint == 2)
				jointValue2.push_back(value);
			else if (joint == 3)
				jointValue3.push_back(value);
			else if (joint == 4)
				jointValue4.push_back(value);
			else if (joint == 5)
				jointValue5.push_back(value);
			
		}
		else
			break;
		count++;
	}

	if (jointValue0.size() == jointValue5.size())
		std::cout << "& The trajectory amount inputed: " << jointValue0.size() / 30 << " Layer, " << jointValue0.size() << " points" << std::endl;

	// HIWIN connection && setting
	HROBOT device_id = open_connection("192.168.1.181", 1, callBack);
	double toolCoor[6];
	int toolNum;

	// parameter setting
	clear_alarm(device_id);
	set_operation_mode(device_id, 1);   // expert
	set_override_ratio(device_id, 100); // %
	set_acc_dec_ratio(device_id, 25);   // 0~100%
	set_lin_speed(device_id, 5);       // 0~2000mm/s  50   150   180	
	set_ptp_speed(device_id, 10);       // 0~100% 20
	set_tool_number(device_id, 15);      // tool number choose

	if (get_motor_state(device_id) == 0)
		set_motor_state(device_id, 1);   // Servo on

	Sleep(1000);
	//
	//int re_type = 0; // socket server
	//char* re_ip = new char[20];
	//int re_port = 5123;
	//int re_bracket = 0; // {}
	//int re_separator = 0; // ,
	//bool re_is_format = false;

	//get_network_config(device_id, re_type, re_ip, re_port, re_bracket, re_separator,re_is_format);
	//
	// diplay 
	toolNum = get_tool_number(device_id);
	get_tool_data(device_id, toolNum, toolCoor);
	std::cout << "# operation mode = " << get_operation_mode(device_id) << std::endl;
	std::cout << "# connection level = " << get_connection_level(device_id) << std::endl;
	std::cout << "# robot lin speed = " << get_lin_speed(device_id) << " mm/s" << std::endl;
	std::cout << "# robot ptp speed = " << get_ptp_speed(device_id) << " %" << std::endl;
	std::cout << "# robot acc ratio = " << get_acc_dec_ratio(device_id) << " %" << std::endl;
	std::cout << "# override ratio = " << get_override_ratio(device_id) << " %" << std::endl;
	std::cout << "# tool number = " << toolNum << std::endl;
	std::cout << "Tool coordinate( " << toolCoor[0] << "," << toolCoor[1] << "," << toolCoor[2] << "," << toolCoor[3] << "," << toolCoor[4] << "," << toolCoor[5] << ")" << std::endl;
	std::system("pause");

	// Robot motion parameter setting
	// model = 1 依百分比平滑, 2 依半徑平滑, 3 依速度平滑
	int mode = 3;
	int smooth = 20;
	double pos[6];

	// inital position 
	double inital_point[6] = { 14.949, 11.449, -23.150, 0, -78.300, 14.949 };
	std::cout << "inital_point." << std::endl;
	std::cout << "【 Robot pos joint 】: ( " << inital_point[0] << "," << inital_point[1] << "," << inital_point[2] << "," << inital_point[3] << "," << inital_point[4] << "," << inital_point[5] << " )" << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
	std::system("pause");  // step by step
	ptp_axis(device_id, 1, inital_point);

	// inter position    model5、8 長方形沒有
	//double inter_point[6] = { -15.322, -1.602, -59.959, -17.305, 62.658, -81.856};   // model8 圓形三角形
	//double inter_point[6] = { -6.008, -10.931, -53.945, -6.629, 65.023, 92.81 };   // 1
	//double inter_point[6] = { -2.461, -14.543, -49.069, -2.745, 63.638, -88.780 };   // 2
	//double inter_point[6] = { -3.678, -11.444, -22.791, 0, -55.767, -3.678 };   // model8 長方形

	double inter_point[6] = { -33.375, -5.65, -55.53, -36.938, 66.261, -73.16 };   // model9
	std::cout << "inter_point." << std::endl;
	std::cout << "【 Robot pos joint 】: ( " << inter_point[0] << "," << inter_point[1] << "," << inter_point[2] << "," << inter_point[3] << "," << inter_point[4] << "," << inter_point[5] << " )" << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
	std::system("pause");  // step by step
	ptp_axis(device_id, 1, inter_point);

	//Sleep(6500);

	int layer_1point_speed = 50;  //60
	int line_1point_speed = 3;	//7
	int layer_eachpoint_speed = 20; //90
	int changeback_line_speed = 15; //30
	int relay_point_speed = 30; //150
	std::cout << "# 該層第一個點速度 = " << layer_1point_speed << " mm/s" << std::endl;
	std::cout << "# 每條軌跡第一個點速度 = " << line_1point_speed << " mm/s" << std::endl;
	std::cout << "# 其他軌跡點速度 = " << layer_eachpoint_speed << " mm/s" << std::endl;
	std::cout << "# 切換下一條軌跡前的速度 = " << changeback_line_speed << " mm/s" << std::endl;
	std::cout << "# 移動到中繼點的速度 = " << relay_point_speed << " mm/s" << std::endl;
	std::system("pause");  // step by step

	// start move to trajectory points
	for (int i = 0 ; i < jointValue0.size(); i++)
	{
		pos[0] = jointValue0[i] * 180.0 / 3.14159265359; // joint1
		pos[1] = jointValue1[i] * 180.0 / 3.14159265359; // joint2
		pos[2] = jointValue2[i] * 180.0 / 3.14159265359; // joint3
		pos[3] = jointValue3[i] * 180.0 / 3.14159265359; // joint4
		pos[4] = jointValue4[i] * 180.0 / 3.14159265359; // joint5
		pos[5] = jointValue5[i] * 180.0 / 3.14159265359; // joint6  

		std::cout << "No." << i << std::endl;
		std::cout << "【 Robot pos joint 】: ( " << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << "," << pos[4] << "," << pos[5] << " )" << std::endl;
		std::cout << "------------------------------------------------------------------------------------" << std::endl;
		//std::system("pause");  // step by step

		//ptp_axis(device_id, 1, pos);
		//lin_pos(device_id, mode, smooth, pos);
		//std::system("pause");  // step by step
		//建立通訊用的txt，連接python的forcedetect 
		//if (i == 0 || i % 10 == 0)
		//{
		//	double first_point_of_line[6];
		//	get_current_joint(device_id, first_point_of_line);
		//	//檔案覆蓋
		//	int a = 1;
		//	std::fstream file1;
		//
		//	file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
		//	file1 << a << " " << a;
		//	file1.close();
		//}
		//else if ((i + 1) % 10 == 0)
		//{
		//	int b = 2;
		//	std::fstream file2;
		//
		//	file2.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
		//	file2 << b << " " << b;
		//	file2.close();
		//}
		//std::system("pause");  // step by step


		// 每一層第一點速度減慢
		if (i == 0 || i % 10 == 0)   //  10
		{

			int arrive_fpoint = 1;

			// 每層第一個點
			if (i == 0 || i % 30 == 0)
			{
				set_lin_speed(device_id, layer_1point_speed);           
			}

			// 其他軌跡第一個點
			else if (i % 10 == 0 && i % 30 != 0)
			{
				set_lin_speed(device_id, line_1point_speed);
			}

			lin_axis(device_id, mode, smooth, pos);

			// 連接sensor，回傳到python
			while (1)
			{
				// 判斷手臂狀態
				if (get_motion_state(device_id) == 1)
				{
					get_motion_state(device_id);
					std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

					int a = 1;
					std::fstream file1;
					file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
					file1 << a ;
					file1.close();
					arrive_fpoint = 0;

					if (arrive_fpoint == 0)
					{
						while (1)
						{
							std::vector<int> returnnum;
							std::ifstream returnfile;
							returnfile.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\return_robot.txt", std::ios::in);

							int python_return;
							while (returnfile >> python_return)
							{
								returnnum.push_back(python_return);
							}

							std::cout << "python_return = " << python_return << std::endl;

							if (python_return == 1)
							{
								// sensor連接成功
								std::cout << "python_return =" << python_return << "------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
								std::cout << "python 回傳成功-----------------------------------------" << std::endl;
								std::cout << "sensor已連接-----------------------------------------" << std::endl;

								break;
							}
						}
					}
					break;

				}
			}
		}
		
		// 每條軌跡最後一點
		else if ((i + 1) % 10 == 0)
		{
			set_lin_speed(device_id, layer_eachpoint_speed);
			lin_axis(device_id, mode, smooth, pos);

			int arrive_lpoint = 1;
			// 斷開sensor，回傳到python
			while (1)
			{
				int robot_arrive;
				robot_arrive = get_motion_state(device_id);
				/*std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;*/
				if (robot_arrive == 1)
				{
					std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;

					int b = 2;
					std::fstream file2;
					file2.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
					file2 << b ;
					file2.close(); 
					std::cout << "sensor已關閉-----------------------------------------" << std::endl;
					arrive_lpoint = 0;
				}
				if (arrive_lpoint == 0)
				{
					break;
				}
					
			}

			// 連接下一條軌跡
			if ((i + 1) % 30 != 0)
			{
				set_lin_speed(device_id, changeback_line_speed);
				double back_to_change[6] = { 0, -10, 0, 0, 0, 0 };
				std::cout << "back_to_change" << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				lin_rel_pos(device_id, mode, smooth, back_to_change);
			}
		}

		// 其他軌跡點
		else
		{
			set_lin_speed(device_id, layer_eachpoint_speed);
			lin_axis(device_id, mode, smooth, pos);
			//system("pause");
		}


		// 每一單層磨完離開-----------------------------------------------
		if ((i + 1) % 30 == 0)    //  10
		{	
			// 最後一層最後一點
			if ((i + 1) == (jointValue0.size()))
			{
				set_lin_speed(device_id, relay_point_speed);
				double back_to_rotate[6] = { -100, -200, 50, 0, 0, 0 };
				std::cout << "back_to_rotate." << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				lin_rel_pos(device_id, mode, smooth, back_to_rotate);
				//std::system("pause");  // step by step
				Sleep(2000);
			}
				
			else
			{
				set_lin_speed(device_id, relay_point_speed);
				double back_to_rotate[6] = { 120, -200, 50, 0, 0, 0 };
				std::cout << "back_to_rotate." << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				lin_rel_pos(device_id, mode, smooth, back_to_rotate);
				//std::system("pause");  // step by step
				Sleep(2000);
			}
				
			// 最後一層切換到另一邊進行細磨
			if ((i + 1) == (jointValue0.size() - 30))
			{
				std::cout << "----進行最後一層細磨----" << std::endl;

				double robot_back[6] = { -33.375, -5.65, -55.53, -36.938, 66.261, -73.16 };
				//double robot_back[6] = { 100, -100, 20, 0, 0, 0 }; 
				std::cout << "robot_back." << std::endl;
				std::cout << "【 Robot pos joint 】: ( " << robot_back[0] << "," << robot_back[1] << "," << robot_back[2] << "," << robot_back[3] << "," << robot_back[4] << "," << robot_back[5] << " )" << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				std::system("pause");  // step by step
				ptp_axis(device_id, 1, robot_back);
				
				double robot_relay_pos[6] = { 31.183, -4.449, -56.887, 34.596, 65.773, -105.804 };
				//double robot_relay_pos[6] = { 14.949, 11.449, -23.150, 0, -78.300, 14.949 };
				std::cout << "relay_point." << std::endl;
				std::cout << "【 Robot pos joint 】: ( " << robot_relay_pos[0] << "," << robot_relay_pos[1] << "," << robot_relay_pos[2] << "," << robot_relay_pos[3] << "," << robot_relay_pos[4] << "," << robot_relay_pos[5] << " )" << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				std::system("pause");  // step by step
				lin_axis(device_id, mode, smooth, robot_relay_pos);
				//ptp_axis(device_id, 1, robot_relay_pos);
				std::system("pause");  // step by step

			}
		}
			
	}

	double robot_back[6] = { 31.183, -4.449, -56.887, 34.596, 65.773, -105.804 };
	//double robot_back[6] = { -60, -170, 20, 0, 0, 0};
	std::cout << "robot_back." << std::endl;
	std::cout << "【 Robot pos joint 】: ( " << robot_back[0] << "," << robot_back[1] << "," << robot_back[2] << "," << robot_back[3] << "," << robot_back[4] << "," << robot_back[5] << " )" << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
	std::system("pause");  // step by step
	lin_axis(device_id, mode, smooth, robot_back);
	//ptp_axis(device_id, 1, robot_back);
	//lin_rel_pos(device_id, mode, smooth, robot_back);
	std::system("pause");  // step by step

	// initial position 
	double robot_initial_pos[6] = {0, 0, 0, 0, -90.0, 0};
	ptp_axis(device_id, 1, robot_initial_pos);

	while (1)
	{
		if (get_motion_state(device_id) == 1)
			break;
	}

	// operation over
	disconnect(device_id);
	std::cout << "Robot working is over.." << std::endl;

	
}

//int main() 
//{
//	SerialCom SixForceAxisSensor;
//	SixForceAxisSensor.OpenSerialCom("COM6");
//	SixForceAxisSensor.SettingSerialCom(115200, 8, ONESTOPBIT, NOPARITY);
//
//
//	////-- Calculating average
//	vector<float> average;
//	vector<float> current;
//	vector<float> reference;
//	float Fresult;
//	cout << endl;
//	average = AverageSensor(SixForceAxisSensor, 50);
//	cout << endl;
//	cout << "平均 " << average[0] << " " << average[1] << " " << average[2] << endl;
//
//	Fresult = resultantForce(average);
//	cout << "平均合力 = " << Fresult << endl;
//
//	ofstream oFile;
//	oFile.open("forcedata.csv", ios::out | ios::trunc);
//
//	for (int i = 0; i <= 2; i++)
//	{
//		cout << i << endl;
//		int run = 0; 
//		while (true)
//		{
//			run = run + 1;
//			current = ReadSensor(SixForceAxisSensor);
//			reference = RefForceSensor(current, average);
//			cout << "瞬間受力 " << reference[0] << "\t" << reference[1] << "\t" << reference[2] << endl;
//
//			float Fresult;
//			Fresult = resultantForce(reference);
//			cout << "合力 = " << Fresult << endl;
//
//
//			oFile << Fresult <<"N"<< " ";
//
//			if (run > 30)
//			{
//				oFile << endl; 
//				break;
//			}
//		}
//	}
//	
//	
//
//}