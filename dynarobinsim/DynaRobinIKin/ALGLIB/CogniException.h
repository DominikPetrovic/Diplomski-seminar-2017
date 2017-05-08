#ifndef __CogniException_h
#define __CogniException_h


#pragma once


#include<string>

class /*DLL_API*/ CException
{
private:
	char *m_message;
public:
	CException(const char* message)
	{
		int size = strlen(message);
		m_message = new char[size + 1];

		memcpy(m_message, message, size + 1);
	}

	CException(const CException& exception)
	{
		int size = strlen(exception.m_message);
		m_message = new char[size + 1];

		memcpy(m_message, exception.m_message, size + 1);
	}
	~CException() { delete [] m_message; }

	char * what() const { return m_message; }

};

#endif

