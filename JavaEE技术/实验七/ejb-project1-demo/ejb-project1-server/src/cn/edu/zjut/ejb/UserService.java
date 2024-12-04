package cn.edu.zjut.ejb;
import javax.ejb.Stateless;
@Stateless
public class UserService implements UserServiceRemote {
    public UserService() { }
    public boolean login(String username, String password){
        if(username.equals("zjut")&&password.equals("zjut")){
            return true;
        }else
            return false;
    }
}