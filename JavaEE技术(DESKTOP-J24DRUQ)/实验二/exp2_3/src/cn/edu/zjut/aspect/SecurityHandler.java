package cn.edu.zjut.aspect;

import org.aspectj.lang.annotation.Before;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Pointcut;

@Aspect
public class SecurityHandler
{
    /**
     * 定义 Pointcut,Pointcut 的名称是 modify，
     * 此方法不能有返回值和参数，该方法只是一个标识
     */
    @Pointcut("execution(* login*(..)) ")
    private void search(){ };

    /**
     * 定义 Advice，标识在那个切入点何处织入此方法
     */
    @Before("search()")
    private void checkSecurity()
    {
        System.out.println("---checkSecurity()---");
    }
}