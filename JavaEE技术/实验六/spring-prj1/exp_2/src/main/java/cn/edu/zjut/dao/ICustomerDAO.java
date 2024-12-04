package cn.edu.zjut.dao;

import cn.edu.zjut.po.Customer;
import org.hibernate.Session;

public interface ICustomerDAO
{
    void save(Customer transientInstance);

    void setSession(Session session);

    void insert(Customer customer);
}