package cn.edu.zjut.app;

import java.util.List;

import cn.edu.zjut.po.Item;
import cn.edu.zjut.service.ItemService;

//public class HibernateTest
//{
//    public static void main(String[] args)
//    {
//        // 创建ItemService实例
//        ItemService itemService = new ItemService();
//
//        // 获取所有商品标题信息
//        System.out.println("All Item Titles:");
//        List<String> itemTitles = itemService.findByHql();
//        for (String title : itemTitles)
//        {
//            System.out.println("Title: " + title);
//        }
//    }
//}

public class HibernateTest
{
    public static void main(String[] args)
    {
        // 创建ItemService实例
        ItemService itemService = new ItemService();

        // 获取所有商品的标题和成本信息
        System.out.println("All Items:");
        List<Object[]> items = itemService.findByHql();
        for (Object[] item : items)
        {
            String title = (String) item[0]; // 第一个元素是标题
            Float cost = (Float) item[1];    // 第二个元素是成本
            System.out.println("Title: " + title + ", Cost: " + cost);
        }
    }
}
