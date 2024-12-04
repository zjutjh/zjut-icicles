package cn.edu.zjut.dao;

import java.util.List;
import java.util.Map;

import org.apache.ibatis.annotations.Mapper;
import cn.edu.zjut.po.MyUser;

@Mapper
public interface UserDao
{
    public MyUser SelectUserById(Integer id);

    public List<MyUser> selectAllUser(Map<String, Object> param);

    public int addUser(MyUser user);

    public int updateUser(MyUser user);

    public int deleteUser(MyUser user);
}