package cn.edu.zjut.ejb;

import javax.ejb.Remote;

@Remote
public interface HelloWorld {

    public String hello(String word);
}