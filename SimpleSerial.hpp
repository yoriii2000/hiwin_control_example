/*
 * SimpleSerial.hpp
 *
 *  Created on: March 28, 2016
 *      Author: Joel Vidal (jolvid@gmail.com)
 *      Version: 0.2.2 (beta)
 */

#ifndef SIMPLESERIAL_H
#define SIMPLESERIAL_H

#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <string>


#define SIMPLESERIAL_DEBUG_INFO true

//DEBUGGING INFO MACROS
#define INFO_DEBUG(X) if(SIMPLESERIAL_DEBUG_INFO) { std::stringstream ss; ss << X; std::cout << ss.str() << std::endl; }


class SimpleSerial
{
public:

	//Short Typedef from Boost
	typedef boost::asio::serial_port_base::flow_control		flow_control;
	typedef boost::asio::serial_port_base::parity			parity;
	typedef boost::asio::serial_port_base::stop_bits		stop_bits;
	typedef boost::asio::serial_port_base::character_size	        character_size;



    SimpleSerial()
    : io_(), serial_(io_), deadlineTimer_(io_), endLineChar_(0)
    {
		//Do nothing.
    }


    //throws boost::system::system_error if cannot open the

    SimpleSerial(std::string port, unsigned int baud_rate)
    : io_(), serial_(io_,port), deadlineTimer_(io_), endLineChar_(0)
    {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }


    //throws boost::system::system_error if cannot open the

	SimpleSerial(std::string port, unsigned int baudRateValue, flow_control::type flowControlValue, parity::type parityValue, stop_bits::type stopBitsValue, character_size characterSizeValue)
		: io_(), serial_(io_,port), deadlineTimer_(io_), endLineChar_(0)
	{
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baudRateValue));
		serial_.set_option(boost::asio::serial_port_base::flow_control( flowControlValue ));
		serial_.set_option(boost::asio::serial_port_base::parity( parityValue ));
		serial_.set_option(boost::asio::serial_port_base::stop_bits( stopBitsValue ));
		serial_.set_option(boost::asio::serial_port_base::character_size(characterSizeValue));
	}


	//return 0 if Ok, otherwise different than 0
	int Open(std::string port, unsigned int baudRateValue, flow_control::type flowControlValue, parity::type parityValue, stop_bits::type stopBitsValue, character_size characterSizeValue)
	{
		if ( isOpen() ) {
			int error = Close();
			if (error) {
				INFO_DEBUG("Error closing the last serial port connection.")
				return -2;
			}
		}

		try {
			serial_.open(port); 
			serial_.set_option(boost::asio::serial_port_base::baud_rate(baudRateValue));
			serial_.set_option(boost::asio::serial_port_base::flow_control( flowControlValue ));
			serial_.set_option(boost::asio::serial_port_base::parity( parityValue ));
			serial_.set_option(boost::asio::serial_port_base::stop_bits( stopBitsValue ));
			serial_.set_option(boost::asio::serial_port_base::character_size(characterSizeValue));
			return 0;
		} catch ( boost::system::system_error e) {
			INFO_DEBUG("Error, Connection fail." << std::endl << "Info:  " << e.what())
			//throw std::exception("Connection fail"); //Throw error
			return -1;
		}
	}

	//
   //Method that checks if a serial conection is openned
	//return 0 if Ok, otherwise different than 0
    //
	bool isOpen()
	{
		return serial_.is_open();
	}


	//
    // Method that checks if a serial conection is openned
	//eturn true if open, otherwise false
    //
	int Close () {
		try {
			serial_.close();
			return 0;
		} catch (boost::system::system_error e) {
			INFO_DEBUG("Error closing serial port connection." << std::endl << "Info:  " << e.what())
			return -1;
		}
	}


	int operator<<( char * msg) { return WriteString(msg); }
	int operator<<( int msg) {std::stringstream ss; ss << msg; return WriteString(ss.str()); }
	int operator<<( long long int msg) { std::stringstream ss; ss << msg; return WriteString(ss.str()); } 
	int operator<<( short int msg) { std::stringstream ss; ss << msg; return WriteString(ss.str()); } 
	int operator<<( float msg) { std::stringstream ss; ss << msg; return WriteString(ss.str()); } 
	int operator<<( double msg) { std::stringstream ss; ss << msg; return WriteString(ss.str()); } 
	int operator<<( char msg) { std::stringstream ss; ss << msg; return WriteString(ss.str()); } 
	int operator<<( std::string& msg) { return WriteString(msg); }




	int operator>>( std::string &msg)
	{
		// write obj to stream
		std::string line;
		int error = ReadLine(line);
		if (!error)
			msg.append(line);
		return error;
	}

    /**+
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    int WriteString(std::string s)
    {

		if (endLineChar_) //if final character differnt than 0 or '\0'
			s.push_back(endLineChar_);

		try {
			boost::asio::write(serial_,boost::asio::buffer(s.c_str(),s.size()));
			//std::cout << "SENT: " << s << std::endl;
			return 0;
		} catch (boost::system::system_error e) {
			INFO_DEBUG("Error writing serial port connection." << std::endl << "Info:  " << e.what())
			return -1;
		}
    }

   
   //Reads until new line (char '\n') is received from the serial device.
   //The '\n' is not included, any '\r' is not tacken in to account
   //param msg Output string with the line
   //return 0 if OK, otherwise different than 0
   //IMPORTANT: Line is readed charcater by character and the timeout is for each of the character before end line
   //TODO: May remove the part of discarting \r charcters???
    int ReadLine(std::string &msg, unsigned int timeout = 100)
    {
		//TODO: change code for this line as it is the same? what to do with \r part?
		//return ReadUntil( msg, '\n', timeout);

		msg.clear();
        char c;

		bool finish = false;
        while (!finish)
        {
			try {

				readErrorFlag_.clear();
				readbytes_ = 0;

				if (timeout != 0) {
					//Set deadline timer
					deadlineTimer_.expires_from_now(boost::posix_time::milliseconds(timeout));
					deadlineTimer_.async_wait( boost::bind(&SimpleSerial::TimeOutCallback,
						this, boost::asio::placeholders::error) );
				}


				//boost::asio::read(serial_,boost::asio::buffer(&c,1));
				boost::asio::async_read(serial_, boost::asio::buffer(&c,1), boost::bind(&SimpleSerial::ReadFinish, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
				io_.reset();
				io_.run(); //Run

				//After run, check the 
				if (readErrorFlag_)
					return -1; //timeout

				if (readbytes_ == 0) //Some times no data is recived so we must continue next loop (IMPORTANT)
					continue;

				switch(c)
				{
				case '\r':
					break;
				case '\n':
					finish = true;
					break;
				default:
					msg += c;
				}
			} catch (boost::system::system_error e) { //throws boost::system::system_error on failure
				INFO_DEBUG("Error reading serial port connection." << std::endl << "Info:  " << e.what())
				return -3;
			}
        }

		return 0;
    }

	int Read( std::string &msg,  unsigned int length = 1, unsigned int timeout = 100 ) {

		msg.clear();

		readErrorFlag_.clear();
		readbytes_ = 0;

		if (timeout != 0) {
			//Set deadline timer
			deadlineTimer_.expires_from_now(boost::posix_time::milliseconds(timeout));
			deadlineTimer_.async_wait( boost::bind(&SimpleSerial::TimeOutCallback,
				this, boost::asio::placeholders::error) );
		}

		try {
			//set async read
			std::vector<char> data(length);
			boost::asio::async_read(serial_, boost::asio::buffer(data, length), boost::bind(&SimpleSerial::ReadFinish, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			io_.reset();
			io_.run(); //Run

			//After run, check the 
			if (readErrorFlag_)
				return -1; //timeout

			if (readbytes_ == 0)
				return -2;

			msg.assign(data.begin(),data.end());

		} catch (boost::system::system_error e) { //throws boost::system::system_error on failure
			INFO_DEBUG("Error reading serial port connection." << std::endl << "Info:  " << e.what())
				return -3; //unknown error
		}

		return 0;
	}

   //Reads until teh character specified is received from the serial device.
   //Final character is not included, any '\r' is not tacken in to account
   //param msg Output string with the line
   //return 0 if OK, otherwise different than 0
   //IMPORTANT: Line is readed charcater by character and the timeout is for each of the character before end line
   //TODO: May remove the part of discarting \r charcters???
	int ReadUntil(std::string &msg, char finalChar = '\n', unsigned int timeout = 100)
    {

		msg.clear();
        char c;

		bool finish = false;
        while (!finish)
        {
			try {

				readErrorFlag_.clear();
				readbytes_ = 0;

				if (timeout != 0) {
					//Set deadline timer
					deadlineTimer_.expires_from_now(boost::posix_time::milliseconds(timeout));
					deadlineTimer_.async_wait( boost::bind(&SimpleSerial::TimeOutCallback,
						this, boost::asio::placeholders::error) );
				}


				//boost::asio::read(serial_,boost::asio::buffer(&c,1));
				boost::asio::async_read(serial_, boost::asio::buffer(&c,1), boost::bind(&SimpleSerial::ReadFinish, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
				io_.reset();
				io_.run(); //Run

				//After run, check the 
				if (readErrorFlag_)
					return -1; //timeout

				if (readbytes_ == 0) //Some times no data is recived so we must continue next loop (IMPORTANT)
					continue;

				if ( c == finalChar ) {
					finish = true;
				} else if ( c != '\r' ) { //Change this line for }else{ if '\r' character need to be considerated
					msg += c;
				}

			} catch (boost::system::system_error e) { //throws boost::system::system_error on failure
				INFO_DEBUG("Error reading serial port connection." << std::endl << "Info:  " << e.what())
				return -3;
			}
        }

		return 0;
    }

	//Decrepetd //To remove!
	/*
	//TODO: PROBLEM the data buffer read everything after the designed endText, so if two mesagte are to near read both
	//      but we just must returm the first one, so that kills de second one
	int ReadUntil( std::string &msg, std::string endText = "\n", unsigned int timeout = 100 ) {

		msg.clear();
		readErrorFlag_.clear();
		readbytes_ = 0;

		if (timeout != 0) {
			//Set deadline timer
			deadlineTimer_.expires_from_now(boost::posix_time::milliseconds(timeout));
			deadlineTimer_.async_wait( boost::bind(&SimpleSerial::TimeOutCallback,
				this, boost::asio::placeholders::error) );
		}

		try {
			//set async read
			boost::asio::streambuf dataBuffer;
			boost::asio::async_read_until(serial_, dataBuffer, endText , boost::bind(&SimpleSerial::ReadFinish, this,boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
			io_.reset();
			io_.run(); //Run

			//After run, check the 
			if (readErrorFlag_)
				return -1; //timeout

			if (readbytes_ == 0)
				return -2;
		
			std::ostringstream ss;
			ss << &dataBuffer; //Pass from asio::streambuf to stringstream
			msg = ss.str();
			msg.resize(readbytes_);


		} catch (boost::system::system_error e) { //throws boost::system::system_error on failure
			INFO_DEBUG("Error reading serial port connection." << std::endl << "Info:  " << e.what())
				return -3; //unknown error
		}
		
		return 0;
	}

    */
	
	/**
     * Set a common used final character for the all message sent
	 * EX: For the end line character '\r' and message "hello" we will have "hello\r"
     * By default no character is used (defined by end of string char '\0\ or 0)
	 * \param c Character to be set for the end of the sent messages
     * \return 0 if OK, otherwise different than 0
     */
	int SetEndLineChar( char c = 0 ) {
		INFO_DEBUG("[INFO] End line common character used! Char num: " << int(c) )
		endLineChar_ = c;
        return 0;
	}


private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
	boost::asio::deadline_timer deadlineTimer_;
	boost::system::error_code readErrorFlag_;
	size_t readbytes_;
	char endLineChar_;

	void ReadFinish (const boost::system::error_code& error,size_t bytesTransferred) {
		readErrorFlag_ = error;
		readbytes_ = bytesTransferred;
		deadlineTimer_.cancel(); //cancel deadline timeout

	}

	void TimeOutCallback (const boost::system::error_code& error) {
		if (error) {
			return;  //If Time out is canceled do nothing
		}
		// The read callback will be called
		// with an error
		serial_.cancel();
		//io_.reset(); //After error we must restart the io service
	}
};

#endif /* SIMPLESERIAL_H */
