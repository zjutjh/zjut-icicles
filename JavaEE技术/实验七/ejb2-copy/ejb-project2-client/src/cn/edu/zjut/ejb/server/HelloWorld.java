package cn.edu.zjut.ejb.server;

import javax.ejb.Remote;

@Remote
public interface HelloWorld {

    public String hello(String word);
}