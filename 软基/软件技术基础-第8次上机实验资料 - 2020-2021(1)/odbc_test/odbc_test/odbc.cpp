#include <iostream>
#include <windows.h>
#include "sql.h"        
#include "sqlext.h"     
#include "sqltypes.h"  
#include "sqlucode.h"  
#include "odbcss.h"    
#include "tchar.h"
#include "stdio.h"
using namespace std;

void main()
{
	SQLHENV henv;
 	SQLRETURN rev;
	SQLHDBC hdbc;
 	char sqlString[255];
 	SQLHSTMT hStmt;

 	SQLCHAR szBuff[128];
 	SQLCHAR CBuff[128];
 	SQLINTEGER  nReadBytes;
 	SQLRETURN     nRC; 

 	cout<<"使用ODBC API函数访问数据库学习\n";
 	rev = SQLAllocHandle(SQL_HANDLE_ENV,SQL_NULL_HANDLE,&henv);
 	if(rev == SQL_SUCCESS_WITH_INFO || rev == SQL_SUCCESS)
 	{
  		cout<<"分配环境成功\n";
 	}
	else
 	{
  		cout<<"分配环境失败\n";
  		return;
 	}
 	rev = SQLSetEnvAttr(henv,SQL_ATTR_ODBC_VERSION,(void *)SQL_OV_ODBC3,0);
 	if(rev == SQL_SUCCESS || rev == SQL_SUCCESS_WITH_INFO)   
 	{   
    	rev=SQLAllocHandle(SQL_HANDLE_DBC,henv,&hdbc);
    	if(rev == SQL_SUCCESS_WITH_INFO || rev == SQL_SUCCESS)
    	{
      		cout<<"分配连接句柄成功\n";
    	}
    	else
    	{
      		cout<<"分配连接句柄失败\n"; 
      		SQLFreeHandle(SQL_HANDLE_ENV, henv);
      		return;
    	}
 	}

 	/*连接数据库，特别注意：sql_zjut_edu为ODBC中配置的数据库名称，需要根据实际配置来替换*/
 	rev = SQLConnect(hdbc,(SQLCHAR *)"JXGL8",SQL_NTS,(SQLCHAR *)"",SQL_NTS,(SQLCHAR *)"",SQL_NTS);
 	if(rev == SQL_SUCCESS_WITH_INFO || rev == SQL_SUCCESS)
 	{
  		cout<<"数据库连接成功\n";
 	}
 	else
 	{
  		cout<<"数据库连接失败\n";
  		SQLFreeHandle(SQL_HANDLE_DBC, hdbc);       
  		SQLFreeHandle(SQL_HANDLE_ENV, henv);
  		system("pause");
  		return;
 	}
 
 	rev = SQLAllocStmt(hdbc, &hStmt); 

 	/*在S表中查询名为李涛的学生学号*/ 
 	sprintf_s(sqlString,"select S# from S where SNAME = '李涛'");
 	rev = SQLExecDirect(hStmt,(SQLCHAR *)sqlString,SQL_NTS);
 	if(rev == SQL_SUCCESS)
 	{
 	 
        while ( (nRC = SQLFetch(hStmt)) != SQL_NO_DATA)
  		{
     		//TODO:获取李涛的学号 ，存入szBuff 
       
            SQLGetData(hStmt, 1, SQL_C_CHAR, &szBuff, 128, &nReadBytes);

   		}
        
   		printf("%s\n",szBuff);
 	}
 	else
 	{
  		cout<<"执行失败!"<<endl;
 	}

 
 	rev = SQLAllocStmt(hdbc, &hStmt); 
 	//TODO:在C表中查询计算机网络的课程号 
    

    sprintf_s(sqlString, "select CLASSH from C where CNAME = '计算机网络'");
    rev = SQLExecDirect(hStmt, (SQLCHAR*)sqlString, SQL_NTS);
 	if(rev == SQL_SUCCESS)
 	{   
  		cout<<"执行SQL语句成功!"<<endl;
        //TODO:获取计算机网络的课程号 ，存入CBuff
        while ((nRC = SQLFetch(hStmt)) != SQL_NO_DATA)
        {
            SQLGetData(hStmt, 1, SQL_C_CHAR, &CBuff, 128, &nReadBytes);
        }

   		printf("%s\n",CBuff);
 	}
 	else
 	{
  		cout<<"执行失败!"<<endl;
 	}

 	rev = SQLAllocStmt(hdbc, &hStmt); 
 
 	//TODO: 更新SC表，将计算机网络课程号和李涛学号对应的分数加2分 

 
 	if(rev == SQL_SUCCESS)
 	{
  		cout<<"执行SQL语句成功!"<<endl;
 	}
 	else
	 	cout<<"执行失败!"<<endl;
  	/*销毁内存*/
  	if (hStmt != SQL_NULL_HSTMT) 
  	{
    	SQLFreeHandle(SQL_HANDLE_STMT, hStmt);
    	hStmt = SQL_NULL_HSTMT;
  	}
  	SQLDisconnect(hdbc);
  	SQLFreeHandle(SQL_HANDLE_DBC, hdbc);       
  	SQLFreeHandle(SQL_HANDLE_ENV, henv);

  	system("pause");
  	return;
}
