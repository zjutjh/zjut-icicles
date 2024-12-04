package cn.edu.zjut.test;

import cn.edu.zjut.dao.UserDao;
import cn.edu.zjut.pojo.Order;
import cn.edu.zjut.pojo.User;
import org.apache.ibatis.io.Resources;
import org.apache.ibatis.session.SqlSession;
import org.apache.ibatis.session.SqlSessionFactory;
import org.apache.ibatis.session.SqlSessionFactoryBuilder;

import java.io.IOException;
import java.io.InputStream;

public class TestManyToOne
{
    public static void main(String[] args)
    {
        try
        {
            // 创建SqlSessionFactory
            String resource = "mybatis-config.xml";
            InputStream inputStream = Resources.getResourceAsStream(resource);
            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(inputStream);

            // 创建SqlSession
            SqlSession sqlSession = sqlSessionFactory.openSession();

            // 获取UserDao接口的实例
            UserDao userDao = sqlSession.getMapper(UserDao.class);

            // 查询用户及其关联的订单
            User user = userDao.getUserById(1);

            // 打印用户信息及订单信息
            System.out.println("User: " + user.getUsername());
            System.out.println("Orders:");
            for (Order order : user.getOrders())
            {
                System.out.println("Order Number: " + order.getOrderNumber());
            }

            // 关闭SqlSession
            sqlSession.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
