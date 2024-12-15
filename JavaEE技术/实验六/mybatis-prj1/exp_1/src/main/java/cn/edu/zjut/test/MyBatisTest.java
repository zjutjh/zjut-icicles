package cn.edu.zjut.test;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import cn.edu.zjut.po.MyUser;
import org.apache.ibatis.io.Resources;
import org.apache.ibatis.session.SqlSession;
import org.apache.ibatis.session.SqlSessionFactory;
import org.apache.ibatis.session.SqlSessionFactoryBuilder;


public class MyBatisTest
{
    public static void main(String[] args)
    {
        try
        {
            InputStream config = Resources.getResourceAsStream("mybatis-config.xml");
            SqlSessionFactory ssf = new SqlSessionFactoryBuilder().build(config);
            SqlSession ss = ssf.openSession();

            // 查询一个用户
            MyUser mu = ss.selectOne("cn.edu.zjut.mapper.UserMapper.selectUserById", 1);
            System.out.println(mu);

            // 添加一个用户
            MyUser addmu = new MyUser();
            addmu.setUname("张三");
            addmu.setUsex("男");
            ss.insert("cn.edu.zjut.mapper.UserMapper.addUser", addmu);

            // 修改一个用户
            MyUser updateUser = new MyUser();
            updateUser.setUid(1); // 设置要修改的用户的ID
            updateUser.setUname("李四");
            updateUser.setUsex("女");
            ss.update("cn.edu.zjut.mapper.UserMapper.updateUser", updateUser);
            // 删除一个用户
            ss.delete("cn.edu.zjut.mapper.UserMapper.deleteUser", 1);

            // 查询所有用户
            List<MyUser> userList = ss.selectList("cn.edu.zjut.mapper.UserMapper.selectAllUsers");
            for (MyUser user : userList)
            {
                System.out.println(user);
            }

            ss.commit();
            ss.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
