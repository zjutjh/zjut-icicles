package cn.edu.zjut.app;

import cn.edu.zjut.po.*;
import cn.edu.zjut.service.UserService;

public class HibernateTest
{
    public static void main(String[] args)
    {
        Customer loginUser = new Customer();
        loginUser.setCustomerId(1);
        loginUser.setAccount("zjut");
        loginUser.setPassword("Zjut");
        UserService uService = new UserService();
        if (uService.login(loginUser))
            System.out.println(loginUser.getAccount() + " login Success !");
        else
            System.out.println(loginUser.getAccount() + " login Fail !");
    }
}