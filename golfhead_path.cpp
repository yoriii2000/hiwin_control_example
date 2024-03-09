#include<windows.h>
#include<math.h>
#include<fstream>
#include<ctime>
#include<iostream>
#include<sstream>
#include<string>
#include<vector>
#include<chrono>
#include <map>

#include<HRSDK.h>
#include<time.h>
#include "ikfast.h"

#include <iomanip>
#include "SixAxis.h"
#include "ReadAndWriteFile.h"

using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::ostringstream;
using std::istringstream;

using namespace std;
using namespace chrono;

HROBOT device_id;

string readFileIntoString(const string& path) {
	auto ss = ostringstream{};
	ifstream input_file(path);
	if (!input_file.is_open()) {
		cerr << "Could not open the file - '"
			<< path << "'" << endl;
		exit(EXIT_FAILURE);
	}
	ss << input_file.rdbuf();
	return ss.str();
}

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

int MH50_JointLimitInspect(double* pose)
{

	// j2
	if (pose[1] > 100 && pose[1] > 0)
		return 0;
	else if (pose[1] < -135 && pose[1] < 0)
		return 0;

	// j3
	if (pose[2] > 190 && pose[2] > 0)
		return 0;
	else if (pose[2] < -80 && pose[2] < 0)
		return 0;

	// j4
	if (pose[3] > 200 && pose[3] > 0)
		return 0;
	else if (pose[3] < -200 && pose[3] < 0)
		return 0;

	// j5
	if (pose[4] > 130 && pose[4] > 0)
		return 0;
	else if (pose[4] < -130 && pose[4] < 0)
		return 0;


	return 1;
}

int euler_change(string xyzfile, string pathfile)
{
	double START, END;

	/////////////////////// ik part //////////////////////////
	double* ikPtr;
	double* ikSolution;
	double ikSelected[6];

	std::ifstream filename(xyzfile, std::ios::in);
	char file[24];
	strcpy(file, xyzfile.c_str());

	int no;
	no = CountLines(file);
	std::cout << xyzfile << " have " << no << " points" << std::endl << std::endl;
	system("pause");

	std::vector<string> items;
	string record;
	int counter = 0;
	int susspointno = 0;
	char delimiter = ' ';

	ofstream oFile;
	oFile.open(pathfile, ios::out | ios::trunc);

	while (std::getline(filename, record))
	{
		double iiiiik[6];

		int determin = 0;

		istringstream line(record);
		while (std::getline(line, record, delimiter))
		{

			items.push_back(record);

			if (determin == 0)
			{
				iiiiik[0] = std::stof(record);
			}
			else if (determin == 1)
			{
				iiiiik[1] = std::stof(record);
			}
			else if (determin == 2)
			{
				iiiiik[2] = std::stof(record);
			}
			else if (determin == 3)
			{
				iiiiik[3] = std::stof(record);
			}
			else if (determin == 4)
			{
				iiiiik[4] = std::stof(record);
			}
			else if (determin == 5)
			{
				iiiiik[5] = std::stof(record);
			}

			determin++;
		}

		//--------------------------------------------------------------
		double ik_in[6] = { 0,0,0,0,0,0 };

		ikPtr = iiiiik;
		ikSolution = ik_in;

		int n = 0;

		double pi = 3.14159265358979323846;
		double pose[6];

		while (ikSolution[0] >= 0)
		{

			ikSolution = ik_euler(ikPtr, n);

			int poseCheck = 0;

			// if ikSolution[0] = -1, nothing to display
			if (ikSolution[0] >= 0)
			{
				pose[0] = ikSolution[1];
				pose[1] = ikSolution[2];
				pose[2] = ikSolution[3];
				pose[3] = ikSolution[4];
				pose[4] = ikSolution[5];
				pose[5] = ikSolution[6];

				poseCheck = MH50_JointLimitInspect(pose);
				poseCheck = 1;

			}

			if (ikSolution[0] == 5)
			{

				ikSelected[0] = ikSolution[1];
				ikSelected[1] = ikSolution[2];
				ikSelected[2] = ikSolution[3];
				ikSelected[3] = ikSolution[4];
				ikSelected[4] = ikSolution[5];
				ikSelected[5] = ikSolution[6];

				std::cout << "targetPos" << counter + 1 << "=" << "{" << ikSolution[1] * pi / 180.0 << "," << ikSolution[2] * pi / 180.0 << "," << ikSolution[3] * pi / 180.0
					<< "," << ikSolution[4] * pi / 180.0 << "," << ikSolution[5] * pi / 180.0 << "," << ikSolution[6] * pi / 180.0 << "}" << endl;
				std::cout << "sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk," << "targetPos" << no << ",targetVel)" << endl;

				oFile << ikSolution[1] * pi / 180.0 << " " << ikSolution[2] * pi / 180.0 << " " << ikSolution[3] * pi / 180.0
					<< " " << ikSolution[4] * pi / 180.0 << " " << ikSolution[5] * pi / 180.0 << " " << ikSolution[6] * pi / 180.0 << endl;

				susspointno++;
			}

			n = n + 1;

		}

		items.clear();
		counter++;
	}
	std::cout << "sussesful point number " << susspointno << endl;

	oFile << "end" << endl;
	std::cout << "end" << endl;
}

// HIWIN robot function
void __stdcall callBack(uint16_t, uint16_t, uint16_t*, int)
{

}

// robot arm part
int main()
{

	//
	string xyzfile = "xyzRxyz/AllxyzRxyz0.xyz";
	string pathfile = "path0.csv";

	euler_change(xyzfile, pathfile);
	system("pause");

	// robot move control

	std::ifstream ImportTrajectory(pathfile);

	if (!ImportTrajectory)
		std::cout << "open " << pathfile << " fail." << std::endl << std::endl;
	else
		std::cout << "open " << pathfile << " success." << std::endl << std::endl;

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

	// HIWIN connection && setting
	HROBOT device_id = open_connection("192.168.1.181", 1, callBack);
	double toolCoor[6];
	int toolNum;

	// parameter setting
	clear_alarm(device_id);
	set_operation_mode(device_id, 1);   // expert
	set_override_ratio(device_id, 100); // %
	set_acc_dec_ratio(device_id, 25);   // 0~100%
	set_lin_speed(device_id, 10);       // 0~2000mm/s  50   150   180	
	set_ptp_speed(device_id, 20);       // 0~100% 20 8
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

	// start position 
	double start_pos[6] = { 0, 0, 0, 0, -90.0, 0 };
	ptp_axis(device_id, 1, start_pos);

	// inital position 
	double inital_point[6] = { 0, 0.841, -15.922, 0, -74.919, 0 };
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

	double inter_point[6] = { -22.838, -5.137, -9.733, 0, -75.130, -22.738 };   // model9
	std::cout << "inter_point." << std::endl;
	std::cout << "【 Robot pos joint 】: ( " << inter_point[0] << "," << inter_point[1] << "," << inter_point[2] << "," << inter_point[3] << "," << inter_point[4] << "," << inter_point[5] << " )" << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
	std::system("pause");  // step by step
	ptp_axis(device_id, 1, inter_point);

	//Sleep(6500);

	int protrusion_1point_speed = 90;  //60 7 90 60 150 150   // 90 20 900 250 250 250
	int line_1point_speed = 20;
	int layer_eachpoint_speed = 900;
	int changeback_line_speed = 300;
	int relay_point_speed = 300;
	int go_to_fine_side = 250;
	std::cout << "# 該層第一個點速度 protrusion_1point_speed = " << protrusion_1point_speed << " mm/s" << std::endl;
	std::cout << "# 每條軌跡第一個點速度 line_1point_speed = " << line_1point_speed << " mm/s" << std::endl;
	std::cout << "# 其他軌跡點速度 layer_eachpoint_speed = " << layer_eachpoint_speed << " mm/s" << std::endl;
	std::cout << "# 切換下一條軌跡前的速度 changeback_line_speed = " << changeback_line_speed << " mm/s" << std::endl;
	std::cout << "# 移動到中繼點的速度 relay_point_speed = " << relay_point_speed << " mm/s" << std::endl;

	//

	int arrive_fpoint = 1;
	// 回傳
	while (1)
	{
		if (get_motion_state(device_id) == 1)
		{
			get_motion_state(device_id);
			std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

			int a = 1;
			std::fstream file1;
			file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
			file1 << a;
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
	// 
	//std::system("pause");  // step by step


	for (int i = 0; i < jointValue0.size(); i++)
	{
		pos[0] = jointValue0[i] * 180.0 / 3.14159265358979323846; // joint1
		pos[1] = jointValue1[i] * 180.0 / 3.14159265358979323846; // joint2
		pos[2] = jointValue2[i] * 180.0 / 3.14159265358979323846; // joint3
		pos[3] = jointValue3[i] * 180.0 / 3.14159265358979323846; // joint4
		pos[4] = jointValue4[i] * 180.0 / 3.14159265358979323846; // joint5
		pos[5] = jointValue5[i] * 180.0 / 3.14159265358979323846; // joint6  

		std::cout << "No." << i << std::endl;
		std::cout << "【 Robot pos joint 】: ( " << jointValue0[i] << "," << jointValue1[i] << "," << jointValue2[i] << "," << jointValue3[i] << "," << jointValue4[i] << "," << jointValue5[i] << " )" << std::endl;
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

		// 第一部分研磨 : 一層一條軌跡
		if (i < (jointValue0.size() - 30 * 3))
		{
			// 每層第一個點回傳python連接sensor
			if (i == 0 || i % 10 == 0)   //  10
			{
				//int arrive_fpoint = 1;

				// 第一層第一個點
				if (i == 0)
				{
					set_lin_speed(device_id, protrusion_1point_speed);      
				}

				// 其他層第一個點
				else
				{
					set_lin_speed(device_id, line_1point_speed);
				}
				
				lin_axis(device_id, mode, smooth, pos);

				// 回傳
				//while (1)
				//{
				//	if (get_motion_state(device_id) == 1)
				//	{
				//		get_motion_state(device_id);
				//		std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;
				//
				//		int a = 1;
				//		std::fstream file1;
				//		file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
				//		file1 << a;
				//		file1.close();
				//		arrive_fpoint = 0;
				//
				//		if (arrive_fpoint == 0)
				//		{
				//			while (1)
				//			{
				//				std::vector<int> returnnum;
				//				std::ifstream returnfile;
				//				returnfile.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\return_robot.txt", std::ios::in);
				//
				//				int python_return;
				//				while (returnfile >> python_return)
				//				{
				//					returnnum.push_back(python_return);
				//				}
				//
				//				std::cout << "python_return = " << python_return << std::endl;
				//
				//				if (python_return == 1)
				//				{
				//					// sensor連接成功
				//					std::cout << "python_return =" << python_return << "------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
				//					std::cout << "python 回傳成功-----------------------------------------" << std::endl;
				//					std::cout << "sensor已連接-----------------------------------------" << std::endl;
				//
				//					break;
				//				}
				//			}
				//		}
				//		break;
				//
				//	}
				//}
			}

			// 該條軌跡最後一點
			else if ((i + 1) % 10 == 0)
			{
				set_lin_speed(device_id, layer_eachpoint_speed);
				lin_axis(device_id, mode, smooth, pos);
				int arrive_lastpoint = 1;
				//while (1)
				//{
				//	int robot_arrive;
				//	robot_arrive = get_motion_state(device_id);
				//	/*std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;*/
				//	if (robot_arrive == 1)
				//	{
				//		std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;

				//		int b = 2;
				//		std::fstream file2;
				//		file2.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
				//		file2 << b;
				//		file2.close();
				//		std::cout << "sensor已關閉-----------------------------------------" << std::endl;
				//		arrive_lastpoint = 0;
				//	}
				//	if (arrive_lastpoint == 0)
				//	{
				//		break;
				//	}
				//}

				// 連接下一層軌跡
				set_lin_speed(device_id, changeback_line_speed);
				double back_to_change[6] = { 0, -10, 0, 0, 0, 0 };
				std::cout << "back_to_change" << std::endl;
				std::cout << "------------------------------------------------------------------------------------" << std::endl;
				lin_rel_pos(device_id, mode, smooth, back_to_change);
				//system("pause");


				// 第一部分最後一點
				if ((i + 1) == (jointValue0.size() - 30 * 3))
				{
					//--------------
					int arrive_lastpoint = 1;
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
							file2 << b;
							file2.close();
							std::cout << "sensor已關閉-----------------------------------------" << std::endl;
							arrive_lastpoint = 0;
						}
						if (arrive_lastpoint == 0)
						{
							break;
						}
					}
					//--------------
					set_lin_speed(device_id, relay_point_speed);
					double back_to_rotate[6] = { 0, -100, 50, 0, 0, 0 };
					std::cout << "back_to_rotate." << std::endl;
					std::cout << "------------------------------------------------------------------------------------" << std::endl;
					lin_rel_pos(device_id, mode, smooth, back_to_rotate);
					std::system("pause");  // step by step
					//Sleep(2000);
					//---------------------------
					int arrive_fpoint = 1;
					// 回傳
					while (1)
					{
						if (get_motion_state(device_id) == 1)
						{
							get_motion_state(device_id);
							std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

							int a = 1;
							std::fstream file1;
							file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
							file1 << a;
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
					//-------------------------
					std::cout << "-----------------------第二部分研磨----------------------------" << std::endl;
				}
			}

			// 其他軌跡點
			else
			{
				set_lin_speed(device_id, layer_eachpoint_speed);
				lin_axis(device_id, mode, smooth, pos);
				//system("pause");
			}
		}

		// 第二部分研磨 : 一層三條軌跡
		else
		{	
			if (i % 10 == 0)
			{	
				int arrive_fpoint = 1;

				//每層第一個點
				if (i == (jointValue0.size() - 30 * 3) ||
					i == (jointValue0.size() - 30 * 2) ||
					i == (jointValue0.size() - 30))
				{
					set_lin_speed(device_id, protrusion_1point_speed);
				}
				// 其他條軌跡第一個點
				else
				{
					set_lin_speed(device_id, line_1point_speed);
				}
				lin_axis(device_id, mode, smooth, pos);

				// 回傳python
				/*while (1)
				{

					if (get_motion_state(device_id) == 1)
					{
						get_motion_state(device_id);
						std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

						int a = 1;
						std::fstream file1;
						file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
						file1 << a;
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
				}*/
			}

			// 每條軌跡最後一點
			else if ((i + 1) % 10 == 0)  // 229 239 249 /259 269 279 /289 299 309 /319 329 339
			{
				set_lin_speed(device_id, layer_eachpoint_speed);
				lin_axis(device_id, mode, smooth, pos);
				//int arrive_lpoint = 1;
				//回傳python
				/*while (1)
				{
					int robot_arrive;
					robot_arrive = get_motion_state(device_id);
					//std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;
					if (robot_arrive == 1)
					{
						std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;

						int b = 2;
						std::fstream file2;
						file2.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
						file2 << b;
						file2.close();
						std::cout << "sensor已關閉-----------------------------------------" << std::endl;
						arrive_lpoint = 0;
					}
					if (arrive_lpoint == 0)
					{
						break;
					}
				}*/
				
				// 連接下一條軌跡
				if ((i + 1) != (jointValue0.size() - 30 * 3) &&
					(i + 1) != (jointValue0.size() - 30 * 2) &&
					(i + 1) != (jointValue0.size() - 30 * 1) &&
					(i + 1) != (jointValue0.size()))
				{
					set_lin_speed(device_id, changeback_line_speed);
					double back_to_change[6] = { 0, -10, 0, 0, 0, 0 };
					std::cout << "back_to_change_line" << std::endl;
					std::cout << "------------------------------------------------------------------------------------" << std::endl;
					lin_rel_pos(device_id, mode, smooth, back_to_change);
					//system("pause");

				}
				// 單層磨完離開-----------------------------------------------
				else
				{					
					//---------------
					int arrive_lpoint = 1;
					while (1)
					{
						int robot_arrive;
						robot_arrive = get_motion_state(device_id);
						//std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;
						if (robot_arrive == 1)
						{
							std::cout << "到達最後一點-----------------------------------------" << robot_arrive << std::endl;

							int b = 2;
							std::fstream file2;
							file2.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
							file2 << b;
							file2.close();
							std::cout << "sensor已關閉-----------------------------------------" << std::endl;
							arrive_lpoint = 0;
						}
						if (arrive_lpoint == 0)
						{
							break;
						}
					}
					//--------------- 
					// 最後一層前切換到另一側進行細磨                    //249 279 309 339
					if ((i + 1) == (jointValue0.size() - 30))  //309
					{
						set_lin_speed(device_id, relay_point_speed);
						double back_to_rotate[6] = { 0, -150, 50, 0, 0, 0 };
						std::cout << "back_to_rough_side." << std::endl;
						std::cout << "------------------------------------------------------------------------------------" << std::endl;
						lin_rel_pos(device_id, mode, smooth, back_to_rotate);
						//-------------
						int arrive_fpoint = 1;
						// 回傳
						while (1)
						{
							if (get_motion_state(device_id) == 1)
							{
								get_motion_state(device_id);
								std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

								int a = 1;
								std::fstream file1;
								file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
								file1 << a;
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
						//-------------
						std::cout << "----進行最後一層細磨----" << std::endl;

						//double rough_side[6] = { -22.838, -5.137, -9.733, 0, -75.130, -22.738 };
						////double robot_back[6] = { 100, -100, 20, 0, 0, 0 }; 
						//std::cout << "rough_side." << std::endl;
						//std::cout << "【 Robot pos joint 】: ( " << rough_side[0] << "," << rough_side[1] << "," << rough_side[2] << "," << rough_side[3] << "," << rough_side[4] << "," << rough_side[5] << " )" << std::endl;
						//std::cout << "------------------------------------------------------------------------------------" << std::endl;
						////std::system("pause");  // step by step
						//ptp_axis(device_id, 1, rough_side);

						//double fine_side[6] = { 25.460, -6.788, -7.920, 0, -75.292, 25.460 };
						////double robot_relay_pos[6] = { 14.949, 11.449, -23.150, 0, -78.300, 14.949 };
						//std::cout << "fine_side." << std::endl;
						//std::cout << "【 Robot pos joint 】: ( " << fine_side[0] << "," << fine_side[1] << "," << fine_side[2] << "," << fine_side[3] << "," << fine_side[4] << "," << fine_side[5] << " )" << std::endl;
						//std::cout << "------------------------------------------------------------------------------------" << std::endl;
						////std::system("pause");  // step by step
						//set_lin_speed(device_id, go_to_fine_side);
						//lin_axis(device_id, mode, smooth, fine_side);
						////ptp_axis(device_id, 1, robot_relay_pos);
						//std::system("pause");  // step by step

					}

					// 該凸點最後一點
					else if ((i + 1) == (jointValue0.size()))    //339
					{
						set_lin_speed(device_id, relay_point_speed);
						double back_to_rotate[6] = { 0, -150, 50, 0, 0, 0 };
						std::cout << "back_to_stop." << std::endl;
						std::cout << "------------------------------------------------------------------------------------" << std::endl;
						lin_rel_pos(device_id, mode, smooth, back_to_rotate);
						//std::system("pause");  // step by step
						Sleep(2000);
					}

					// 其他層軌跡最後一點
					else									//249 279
					{                
						set_lin_speed(device_id, relay_point_speed); 
						double back_to_rotate[6] = { 0, -100, 50, 0, 0, 0 };
						std::cout << "back_to_change_layer." << std::endl;
						std::cout << "------------------------------------------------------------------------------------" << std::endl;
						lin_rel_pos(device_id, mode, smooth, back_to_rotate);
						//----------------
						int arrive_fpoint = 1;
						// 回傳
						while (1)
						{
							if (get_motion_state(device_id) == 1)
							{
								get_motion_state(device_id);
								std::cout << "robot state =  " << get_motion_state(device_id) << std::endl;

								int a = 1;
								std::fstream file1;
								file1.open("C:\\Users\\User\\Desktop\\陳昱廷\\Layering\\connect_sensor.txt", std::ios::out);
								file1 << a;
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
						//----------------- 
						//std::system("pause");  // step by step
						//Sleep(2000);
					}
				}
			}

			// 其他軌跡點
			else
			{
				set_lin_speed(device_id, layer_eachpoint_speed);
				lin_axis(device_id, mode, smooth, pos);
				//system("pause");
			}
		}
	}

	std::cout << "----研磨結束----" << std::endl;

	set_lin_speed(device_id, relay_point_speed);
	double robot_back[6] = { 0, 0.841, -15.922, 0, -74.919, 0 };
	//double robot_back[6] = { -60, -170, 20, 0, 0, 0};
	std::cout << "robot_back." << std::endl;
	std::cout << "【 Robot pos joint 】: ( " << robot_back[0] << "," << robot_back[1] << "," << robot_back[2] << "," << robot_back[3] << "," << robot_back[4] << "," << robot_back[5] << " )" << std::endl;
	std::cout << "------------------------------------------------------------------------------------" << std::endl;
	//std::system("pause");  // step by step
	lin_axis(device_id, mode, smooth, robot_back);
	std::system("pause");  // step by step
		
	// initial position 
	double robot_initial_pos[6] = { 0, 0, 0, 0, -90.0, 0 };
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
