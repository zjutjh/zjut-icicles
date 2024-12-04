package cn.edu.zjut.app;

import cn.edu.zjut.bean.IShoppingCart;
import org.springframework.context.ApplicationContext;
import org.springframework.context.support.ClassPathXmlApplicationContext;
import cn.edu.zjut.bean.IItemOrder;

import java.util.*;

//Properties
public class SpringBeanTest
{
    public static void main(String[] args)
    {
        ApplicationContext ctx = new ClassPathXmlApplicationContext("applicationContext.xml");

        IShoppingCart shoppingcart = (IShoppingCart) ctx.getBean("shoppingcart");

        Properties itemsProperties = shoppingcart.getItemsOrdered();
        Enumeration<?> propertyNames = itemsProperties.propertyNames();
        while (propertyNames.hasMoreElements())
        {
            String key = (String) propertyNames.nextElement();
            String itemOrderBeanId = itemsProperties.getProperty(key);

            IItemOrder value = (IItemOrder) ctx.getBean(itemOrderBeanId);

            System.out.println("Properties");
            System.out.println("书名：" + value.getItem().getTitle());
            System.out.println("数量：" + value.getNumItems());
        }
    }

}

//Map
//public class SpringBeanTest
//{
//    public static void main(String[] args)
//    {
//        ApplicationContext ctx = new ClassPathXmlApplicationContext("applicationContext.xml");
//
//        IShoppingCart shoppingcart = (IShoppingCart) ctx.getBean("shoppingcart");
//
//        Map<String, IItemOrder> itemMap = shoppingcart.getItemsOrdered();
//        Iterator<Map.Entry<String, IItemOrder>> iterator = itemMap.entrySet().iterator();
//        while (iterator.hasNext())
//        {
//            Map.Entry<String, IItemOrder> entry = iterator.next();
//            String key = entry.getKey();
//            IItemOrder value = entry.getValue();
//            System.out.println("Map");
//            System.out.println("书名：" + value.getItem().getTitle());
//            System.out.println("数量：" + value.getNumItems());
//        }
//    }
//}

//Set
//public class SpringBeanTest
//{
//    public static void main(String[] args)
//    {
//        ApplicationContext ctx = new ClassPathXmlApplicationContext("applicationContext.xml");
//
//        IShoppingCart shoppingcart = (IShoppingCart) ctx.getBean("shoppingcart");
//
//        Set<IItemOrder> itemSet = shoppingcart.getItemsOrdered(); // 假设shoppingcart是你的ShoppingCart对象
//        Iterator<IItemOrder> iterator = itemSet.iterator();
//        while (iterator.hasNext())
//        {
//            IItemOrder itemOrder = iterator.next();
//            System.out.println("Set");
//            System.out.println("书名：" + itemOrder.getItem().getTitle());
//            System.out.println("数量：" + itemOrder.getNumItems());
//        }
//
//    }
//}

// List
//public class SpringBeanTest
//{
//    public static void main(String[] args)
//    {
//        ApplicationContext ctx = new ClassPathXmlApplicationContext("applicationContext.xml");
//
//        IShoppingCart shoppingcart = (IShoppingCart) ctx.getBean("shoppingcart");
//
//        IItemOrder itemorder1 = (IItemOrder) shoppingcart.getItemsOrdered().get(0);
//        IItemOrder itemorder2 = (IItemOrder) shoppingcart.getItemsOrdered().get(1);
//
//        System.out.println("List");
//
//        System.out.println("书名：" + itemorder1.getItem().getTitle());
//        System.out.println("数量：" + itemorder1.getNumItems());
//
//        System.out.println("书名：" + itemorder2.getItem().getTitle());
//        System.out.println("数量：" + itemorder2.getNumItems());
//
//    }
//}
