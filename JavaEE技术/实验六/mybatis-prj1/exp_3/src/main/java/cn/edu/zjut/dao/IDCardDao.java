package cn.edu.zjut.dao;

import org.apache.ibatis.annotations.Mapper;
import cn.edu.zjut.po.IDcard;

public interface IDCardDao
{
    public IDcard selectCodeById(Integer i);
}