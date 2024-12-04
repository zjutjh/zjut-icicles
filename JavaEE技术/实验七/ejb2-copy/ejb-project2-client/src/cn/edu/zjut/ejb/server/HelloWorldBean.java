package cn.edu.zjut.ejb.server;

import javax.ejb.Stateless;

@Stateless(name = "HelloWorldEJB")
public class HelloWorldBean implements HelloWorld{
    public HelloWorldBean() {
    }

    @Override
    public String hello(String word) {
        return "hello " + word;
    }
}