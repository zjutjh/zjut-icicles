package cn.edu.zjut.app;

import cn.edu.zjut.dao.*;
import cn.edu.zjut.po.*;
import cn.edu.zjut.service.*;
import cn.edu.zjut.util.HibernateUtil;

import java.util.List;

//public class HibernateTest {
//    public static void main(String[] args) {
//        // 创建 ItemService 对象
//        ItemService itemService = new ItemService();
//
//        // 获取所有商品信息
//        List<Item> items = itemService.getAllItems();
//
//        // 遍历并打印商品信息
//        for (Item item : items) {
//            System.out.println("ISBN号: " + item.getItemID());
//            System.out.println("书名: " + item.getTitle());
//            System.out.println("说明: " + item.getDescription());
//            System.out.println("单价: " + item.getCost());
//            System.out.println();
//        }
//    }
//}

//public class HibernateTest
//{
//    public static void main(String[] args)
//    {
//        ItemService itemService = new ItemService();
//        List<Item> items = itemService.getAllItems();
//
//        System.out.println("书本编号\t\t书名");
//        System.out.println("===============================");
//        for (Item item : items)
//        {
//            ItemPK itemPK = item.getIpk();
//            System.out.println(itemPK.getItemID() + "\t" + itemPK.getTitle());
//        }
//    }
//}

public class HibernateTest
{
    public static void main(String[] args)
    {
        // 创建一个用户对象
        ContactInfo contactInfo = new ContactInfo("1234567890", "user@example.com", "123 Main St", "12345", "Fax123");
        Customer newUser = new Customer(0, "newuser", "password123", "New User", true, null, contactInfo);

        UserService userService = new UserService();

        // 注册用户
        boolean registrationSuccess = userService.register(newUser);

        if (registrationSuccess)
        {
            System.out.println("User registration successful!");
        }
        else
        {
            System.out.println("User registration failed.");
        }

        // 关闭 Hibernate 会话
        HibernateUtil.closeSession();
    }
}
