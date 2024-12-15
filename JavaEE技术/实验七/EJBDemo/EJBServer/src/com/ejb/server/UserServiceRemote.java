package com.ejb.server;
import javax.ejb.Remote;

@Remote
public interface UserServiceRemote {
    public boolean login(String username, String password);
}