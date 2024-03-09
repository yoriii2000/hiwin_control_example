#pragma once
#include "SerialCom.h"
#include<vector>
#include<math.h>


vector<float> ReadSensor(SerialCom SixForceAxisSensor)
{

	// =================================================================
	string ReadResult;

	// ------- Define the elements in the vector (Here we want define which one is Fx, Fy, Fz, Mx, My, Mz from the elements above)------------------
	float Fx, Fy, Fz, Mx, My, Mz;


	while (TRUE)
	{
		ReadResult = SixForceAxisSensor.ReadSerialCom();
		//cout << "SerialCom Read: " << ReadResult << endl;

		vector<string>SerialComValue = SixForceAxisSensor.GetValueSerialCom(ReadResult);	// Result is: [[1],[N],[2],[N],....]

		int LengthOfSerialComValue = SerialComValue.size() - 1;			// Count how many elements inside the vector above


	
		// Sometimes the sensor didn't work properly (The datas are given less than normal, suppose only 3 datas), so we have to make sure in this condition.
		// Is set as 12 because the number of elements in the vector above is 12.
		if (LengthOfSerialComValue == 12)
		{
			// Define the F and M from the read values 
			for (int i = 0; i < LengthOfSerialComValue; i++)
			{
				Fx = stof(SerialComValue[0]);	// Fx value is stored on vector number 0. "stof" is string to float (because the value which is read from serial com is string format)
				Fy = stof(SerialComValue[2]);
				Fz = stof(SerialComValue[4]);
				Mx = stof(SerialComValue[6]);
				My = stof(SerialComValue[8]);
				Mz = stof(SerialComValue[10]);
			}
			// Check if the defined values are correct
			//cout << "Fx: " << Fx << endl;
			//cout << "Fy: " << Fy << endl;
			//cout << "Fz: " << Fz << endl;
			//cout << "Mx: " << Mx << endl;
			//cout << "My: " << My << endl;
			//cout << "Mz: " << Mz << endl;

			break; // If there's error, the sensor will read again
		}

		// If the sensor didn't work properly as mentioned before:
		else
		{
			cout << "6 Axis Force Sensor gives values less than 6 (Fx, Fy, Fz, Mx, My, Mz)" << endl;
		}
	};
	// ================================================================================

	vector<float>SensorOutput = { Fx,Fy,Fz,Mx,My,Mz };
	return SensorOutput;
}

vector<float> AverageSensor(SerialCom SixAxisSensorCom, int iterationMax)
{
	vector<float>ScanOutput;
	float Fx, Fy, Fz, Mx, My, Mz;
	float Fx_sum=0, Fy_sum=0, Fz_sum=0, Mx_sum=0, My_sum=0, Mz_sum=0;
	for (int iteration = 0; iteration < iterationMax; iteration++)
	{
		cout << "\niteration:" << iteration << endl;
		ScanOutput = ReadSensor(SixAxisSensorCom);

		Fx = ScanOutput[0];
		Fy = ScanOutput[1];
		Fz = ScanOutput[2];
		Mx = ScanOutput[3];
		My = ScanOutput[4];
		Mz = ScanOutput[5];

		// SUMMARIZE
		Fx_sum = Fx + Fx_sum;
		Fy_sum = Fy + Fy_sum;
		Fz_sum = Fz + Fz_sum;
		Mx_sum = Mx + Mx_sum;
		My_sum = My + My_sum;
		Mz_sum = Mz + Mz_sum;
	}
	// AVERAGE
	float Fx_avg, Fy_avg, Fz_avg, Mx_avg, My_avg, Mz_avg;
	Fx_avg = Fx_sum / iterationMax;
	Fy_avg = Fy_sum / iterationMax;
	Fz_avg = Fz_sum / iterationMax;
	Mx_avg = Mx_sum / iterationMax;
	My_avg = My_sum / iterationMax;
	Mz_avg = Mz_sum / iterationMax;

	cout << "Average: " << Fx_avg << "\t" << Fy_avg << "\t" << Fz_avg << "\t" << Mx_avg << "\t" << My_avg << "\t" << Mz_avg << "\t" << endl;

	vector<float> Average = { Fx_avg ,Fy_avg ,Fz_avg ,Mx_avg ,My_avg ,Mz_avg };

	return Average;
}

float resultantForce(vector<float>average)
{
	float Faverage;
	Faverage = sqrt(pow(average[0], 2) + pow(average[1], 2) + pow(average[2], 2));

	return Faverage;
}

vector<float>RefForceSensor(vector<float> CurrentValue, vector<float> AverageValue)
{

	float Fx_ref, Fy_ref, Fz_ref, Mx_ref, My_ref, Mz_ref;
	Fx_ref = (CurrentValue[0] - AverageValue[0]) ;
	Fy_ref = (CurrentValue[1] - AverageValue[1]) ;
	Fz_ref = (CurrentValue[2] - AverageValue[2])  ;
	Mx_ref = (CurrentValue[3] - AverageValue[3])  ;
	My_ref = (CurrentValue[4] - AverageValue[4])  ;
	Mz_ref = (CurrentValue[5] - AverageValue[5])  ;

	//cout << "Ref: "<< Fx_ref << "\t" << Fy_ref << "\t" << Fz_ref << "\t" << Mx_ref << "\t" << My_ref << "\t" << Mz_ref <<endl;

	vector<float>RefForceValue = { Fx_ref, Fy_ref, Fz_ref, Mx_ref, My_ref, Mz_ref };
	return RefForceValue;
}