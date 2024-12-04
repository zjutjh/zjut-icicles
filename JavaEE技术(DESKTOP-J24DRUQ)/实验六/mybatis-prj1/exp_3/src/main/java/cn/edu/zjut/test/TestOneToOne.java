package cn.edu.zjut.test;

import cn.edu.zjut.dao.PersonDao;
import cn.edu.zjut.pojo.SelectPersonById;
import cn.edu.zjut.po.Person;
import org.apache.ibatis.io.Resources;
import org.apache.ibatis.session.SqlSession;
import org.apache.ibatis.session.SqlSessionFactory;
import org.apache.ibatis.session.SqlSessionFactoryBuilder;

import java.io.IOException;
import java.io.InputStream;

public class TestOneToOne
{
    public static void main(String[] args)
    {
        try
        {
            // 加载 MyBatis 配置文件
            String resource = "mybatis-config.xml";
            InputStream inputStream = Resources.getResourceAsStream(resource);

            // 创建 SqlSessionFactory
            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(inputStream);

            // 创建 SqlSession
            SqlSession sqlSession = sqlSessionFactory.openSession();

            // 获取 PersonDao 接口的实例
            PersonDao personDao = sqlSession.getMapper(PersonDao.class);

            // 使用具体实现类执行查询操作
            Person p1 = personDao.selectPersonById1(1);
            System.out.println(p1);
            System.out.println("=======================");

            Person p2 = personDao.selectPersonById2(1);
            System.out.println(p2);
            System.out.println("=======================");

            SelectPersonById p3 = personDao.selectPersonById3(1);
            System.out.println(p3);

            // 关闭 SqlSession
            sqlSession.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
