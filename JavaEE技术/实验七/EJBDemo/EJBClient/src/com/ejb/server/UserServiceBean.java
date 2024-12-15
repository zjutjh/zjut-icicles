package com.ejb.server;

import javax.ejb.Stateless;

@Stateless
public class UserServiceBean implements UserServiceRemote {
    public UserServiceBean() { }
    public boolean login(String username, String password){
        if(username.equals("zjut")&&password.equals("zjut")){
            return true;
        }else
            return false;
    }
}