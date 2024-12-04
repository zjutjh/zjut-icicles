package cn.edu.zjut.dao;

import org.apache.ibatis.annotations.Mapper;
import cn.edu.zjut.po.Person;
import cn.edu.zjut.pojo.SelectPersonById;

public interface PersonDao
{
    public Person selectPersonById1(Integer id);

    public Person selectPersonById2(Integer id);

    public SelectPersonById selectPersonById3(Integer id);
}