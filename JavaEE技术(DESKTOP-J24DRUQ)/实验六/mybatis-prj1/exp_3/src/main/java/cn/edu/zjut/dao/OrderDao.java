package cn.edu.zjut.dao;

import cn.edu.zjut.pojo.Order;

public interface OrderDao {
    // 根据订单ID查询订单信息
    Order getOrderById(int id);
}
