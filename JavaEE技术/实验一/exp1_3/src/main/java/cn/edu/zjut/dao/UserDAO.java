package cn.edu.zjut.dao;

import java.sql.*;
import javax.sql.*;
import javax.naming.*;

import cn.edu.zjut.model.UserBean;

public class UserDAO
{
    private static final String GET_ONE_SQL = "SELECT * FROM usertable WHERE username=? AND password=? AND type=?";
    private static final String INSERT_SQL = "INSERT INTO usertable (username, password, type) VALUES (?, ?, ?)";

    public UserDAO()
    {
    }

    public Connection getConnection()
            throws SQLException
    {
        Connection conn = null;
        String driver = "org.postgresql.Driver";
        String dburl = "jdbc:postgresql://127.0.0.1:5432/mydb";
        String username = "dbuser"; //数据库登录用户名
        String password = "dbpassword"; //数据库登录密码
        try
        {
            Class.forName(driver); //加载数据库驱动程序
            conn = DriverManager.getConnection(dburl, username, password);
        }
        catch (Exception e)
        {
            throw new SQLException("Database connection error.", e);
        }
        return conn;
    }


    public boolean searchUser(UserBean user)
    {
        Connection conn = null;
        PreparedStatement pstmt = null;
        ResultSet rst = null;
        try
        {
            conn = getConnection();
            pstmt = conn.prepareStatement(GET_ONE_SQL);
            pstmt.setString(1, user.getUsername());
            pstmt.setString(2, user.getPassword());
            pstmt.setInt(3, user.getType());
            rst = pstmt.executeQuery();
            if (rst.next())
            {
                return true;
            }
        }
        catch (SQLException se)
        {
            se.printStackTrace();
            return false;
        }
        finally
        {
            try
            {
                if (pstmt != null)
                {
                    pstmt.close();
                }
                if (conn != null)
                {
                    conn.close();
                }
            }
            catch (SQLException se)
            {
                se.printStackTrace();
            }
        }
        return false;
    }

    public boolean insert(UserBean user)
    {
        Connection conn = null;
        PreparedStatement pstmt = null;
        try
        {
            conn = getConnection();
            pstmt = conn.prepareStatement(INSERT_SQL);
            pstmt.setString(1, user.getUsername());
            pstmt.setString(2, user.getPassword());
            pstmt.setInt(3, user.getType()); // 假设 UserBean 类中的 getType() 方法返回用户类型的字符串值
            int rowsAffected = pstmt.executeUpdate();
            return rowsAffected > 0; // 如果有行受到影响，表示插入成功
        }
        catch (SQLException se)
        {
            se.printStackTrace();
            return false;
        }
        finally
        {
            // 关闭资源
            // 省略部分代码
        }
    }
}