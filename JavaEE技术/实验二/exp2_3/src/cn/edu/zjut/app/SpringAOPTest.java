package cn.edu.zjut.app;

import org.springframework.context.ApplicationContext;
import org.springframework.context.support.ClassPathXmlApplicationContext;
import cn.edu.zjut.bean.UserBean;
import cn.edu.zjut.service.IUserService;

public class SpringAOPTest
{
    public static void main(String[] args)
    {
        //创建 Spring 容器
        ApplicationContext ctx = new ClassPathXmlApplicationContext("applicationContext.xml");
        UserBean loginUser = new UserBean();
        loginUser.setUsername("SPRING");
        loginUser.setPassword("SPRING");
        //获取 UserService 实例
        IUserService userService = (IUserService) ctx.getBean("userService");
        userService.login(loginUser);
    }
}


