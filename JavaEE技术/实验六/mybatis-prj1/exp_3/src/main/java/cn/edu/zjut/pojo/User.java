package cn.edu.zjut.pojo;

import java.util.List;

public class User
{
    private int id;
    private String username;
    private List<Order> orders; // 一对多关系，一个用户可以有多个订单

    // 其他属性和方法的定义...

    public int getId()
    {
        return id;
    }

    public void setId(int id)
    {
        this.id = id;
    }

    public String getUsername()
    {
        return username;
    }

    public void setUsername(String username)
    {
        this.username = username;
    }

    public List<Order> getOrders()
    {
        return orders;
    }

    public void setOrders(List<Order> orders)
    {
        this.orders = orders;
    }
}
