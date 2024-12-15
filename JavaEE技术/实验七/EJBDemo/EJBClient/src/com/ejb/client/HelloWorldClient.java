package com.ejb.client;

import com.ejb.server.HelloWorld;

import javax.naming.Context;
import javax.naming.InitialContext;
import javax.naming.NamingException;
import java.util.Properties;

public class HelloWorldClient {
    public static void main(String[] args) throws NamingException {
        Properties props = new Properties();
        props.put(Context.URL_PKG_PREFIXES,"org.jboss.ejb.client.naming");
        props.put("jboss.naming.client.ejb.context",true);
        try {
            InitialContext ctx = new InitialContext(props);

            String appName = "";
            String moduleName = "EJBServer_war_exploded";
            String distinctName = "";
            String beanName = "HelloWorldEJB";
            String viewClassName = HelloWorld.class.getName();
            String namespace = "ejb:" + appName + "/" + moduleName + "/" + distinctName + "/" + beanName + "!" + viewClassName;
            //String namespace = "ejb:/EJBServer_war_exploded/HelloWorldEJB!com.ejb.server.HelloWorld";
            HelloWorld helloworld = (HelloWorld) ctx.lookup(namespace);
            System.out.println(helloworld.hello("New World"));
        } catch (NamingException e) {
            e.printStackTrace();
        }

    }
}