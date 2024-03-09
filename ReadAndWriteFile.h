//	Author		: Aldi Prasetyo
//	Nationality	: Indonesia
//	Email		: Chaoliang94@gmail.com


#include <iostream>
#include <fstream>
#include <sstream> //For Stringstream
#include <string>
#include <vector>
using namespace std;



// ================= Function: Splitting string with delimiter =========================
template <typename Out>
void split(const string &s, char delim, Out result)
{
	istringstream iss(s);
	string item;
	while (getline(iss, item, delim))
	{
		if (!item.empty())
		{
			*result++ = item;
		}

	}
}
vector<string> split(const string &s, char delim)
{
	vector<string> elems;
	split(s, delim, back_inserter(elems));
	return elems;
}
// ================= ====================== END ============================ =========================




vector<vector<string>> ReadFile(string FileName, char SplitDelim = ',')
{
	fstream file;
	file.open(FileName, ios::in);								//open a file to perform read operation using file object

	vector<vector<string>>DataPerLine;
	string StringData;

	if (file.is_open())											//checking whether the file is open
	{
		cout << "Read file: " << FileName << endl;
		while (getline(file, StringData))						//read data from file object and put it into string line per line.
		{
			//cout << StringData << "\n";						// print the data of the string
			DataPerLine.push_back(split(StringData, SplitDelim));		// take the each value in the string data with delimiter "," 
		}
		file.close();											//close the file object.
	}
	else
	{
		cout << "Unable to read file: " << FileName << endl;
	}




	////-------------- JUST FOR CHECKING ------------------------------
	for (int line = 0; line < DataPerLine.size(); line++)
	{
		for (int ElementInLine = 0; ElementInLine < DataPerLine[line].size(); ElementInLine++)
		{
			cout << DataPerLine[line][ElementInLine] << "\t";
		}
		cout << endl;
	}

	return DataPerLine;
};


void WriteFile(string FileName, vector<vector<string>> text)
{
	fstream file;
	file.open(FileName, ios::out);  // open a file to perform write operation using file object
	if (file.is_open()) //checking whether the file is open {
	{
		for (int data_no = 0; data_no < text.size(); data_no++)
		{
			for (int element_no = 0; element_no < text[data_no].size(); element_no++)
			{
				file << text[data_no][element_no]; //inserting text
			}
			file << "\n";
		}

		file.close(); //close the file object
	}
	else
	{
		cout << "Failed to write the file" << endl;
	};
};
