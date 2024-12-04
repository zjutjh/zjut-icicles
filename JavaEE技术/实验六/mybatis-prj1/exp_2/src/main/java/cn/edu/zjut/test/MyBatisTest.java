package cn.edu.zjut.test;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cn.edu.zjut.dao.UserDao;
import cn.edu.zjut.pojo.MapUser;
import cn.edu.zjut.pojo.SelectUserParam;
import org.apache.ibatis.io.Resources;
import org.apache.ibatis.session.SqlSession;
import org.apache.ibatis.session.SqlSessionFactory;
import org.apache.ibatis.session.SqlSessionFactoryBuilder;
import cn.edu.zjut.po.MyUser;

//public class MyBatisTest
//{
//    public static void main(String[] args)
//    {
//        try
//        {
//            String resource = "mybatis-config.xml"; // MyBatis 配置文件路径
//            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(Resources.getResourceAsReader(resource));
//            SqlSession ss = sqlSessionFactory.openSession();
//
//            MapUser mapUser = new MapUser();
//            mapUser.setM_uid(1); // 设置用户ID
//            mapUser.setM_uname("NewName"); // 设置新的用户名
//            mapUser.setM_usex("NewSex");
//
//            // 插入操作
//            int insertCount = ss.insert("cn.edu.zjut.mapper.UserMapper.insertMapUser", mapUser);
//            System.out.println("插入影响的行数: " + insertCount);
//
//            // 更新操作
//            int updateCount = ss.update("cn.edu.zjut.mapper.UserMapper.updateMapUser", mapUser);
//            System.out.println("更新影响的行数: " + updateCount);
//
//            // 删除操作
//            int deleteCount = ss.delete("cn.edu.zjut.mapper.UserMapper.deleteMapUser", mapUser);
//            System.out.println("删除影响的行数: " + deleteCount);
//
//            ss.commit(); // 提交事务
//            ss.close();
//        }
//        catch (IOException e)
//        {
//            e.printStackTrace();
//        }
//    }
//}

//public class MyBatisTest
//{
//    public static void main(String[] args)
//    {
//        try
//        {
//            String resource = "mybatis-config.xml"; // MyBatis 配置文件路径
//            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(Resources.getResourceAsReader(resource));
//            SqlSession ss = sqlSessionFactory.openSession();
//
//            List<MapUser> list = ss.selectList("cn.edu.zjut.mapper.UserMapper.selectResultMap");
//            for (MapUser mapUser : list)
//            {
//                System.out.println(mapUser);
//            }
//
//            ss.close();
//        }
//        catch (IOException e)
//        {
//            e.printStackTrace();
//        }
//    }
//}
//public class MyBatisTest {
//    public static void main(String[] args) {
//        try {
//            InputStream config = Resources.getResourceAsStream("mybatis-config.xml");
//            SqlSessionFactory ssf = new SqlSessionFactoryBuilder().build(config);
//            SqlSession ss = ssf.openSession();
//
//            Map<String, Object> map = new HashMap<>();
//            map.put("u_name", "陈");
//            map.put("u_sex", "男");
//
//            List<MyUser> list = ss.selectList("cn.edu.zjut.mapper.UserMapper.selectUserByNameAndSex", map);
//
//            for (MyUser myUser : list) {
//                System.out.println(myUser);
//            }
//
//            ss.commit();
//            ss.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
//}

public class MyBatisTest
{
    public static void main(String[] args)
    {
        try
        {
            String resource = "mybatis-config.xml"; // MyBatis 配置文件路径
            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(Resources.getResourceAsReader(resource));
            SqlSession sqlSession = sqlSessionFactory.openSession();

            Map<String, Object> map = new HashMap<>();
            map.put("u_name", "陈");
            map.put("u_sex", "男");

            UserDao userDao = sqlSession.getMapper(UserDao.class);
            List<MyUser> list = userDao.selectAllUser(map);


            for (MyUser myUser : list)
            {
                System.out.println(myUser);
            }

            sqlSession.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}

//public class MyBatisTest
//{
//    public static void main(String[] args)
//    {
//        try
//        {
//            String resource = "mybatis-config.xml"; // MyBatis 配置文件路径
//            SqlSessionFactory sqlSessionFactory = new SqlSessionFactoryBuilder().build(Resources.getResourceAsReader(resource));
//            SqlSession ss = sqlSessionFactory.openSession();
//
//            SelectUserParam su = new SelectUserParam();
//            su.setU_name("陈");
//            su.setU_sex("男");
//            List<Map<String, Object>> list = ss.selectList("cn.edu.zjut.mapper.UserMapper.selectAllUser");
//            for (Map<String, Object> userMap : list)
//            {
//                for (Map.Entry<String, Object> entry : userMap.entrySet())
//                {
//                    String fieldName = entry.getKey();
//                    Object fieldValue = entry.getValue();
//                    System.out.println(fieldName + ": " + fieldValue);
//                }
//                System.out.println();
//            }
//
//            ss.close();
//        }
//        catch (IOException e)
//        {
//            e.printStackTrace();
//        }
//    }
//}

