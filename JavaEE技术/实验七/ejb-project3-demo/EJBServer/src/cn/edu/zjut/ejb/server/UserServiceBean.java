package cn.edu.zjut.ejb.server;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import javax.ejb.Stateless;

@Stateless(name = "UserServiceEJB")
public class UserServiceBean implements UserServiceRemote {
    private static final String CORRECT_USERNAME = "zjut";
    private final String CORRECT_PASSWORD_HASH;

    public UserServiceBean() {
        this.CORRECT_PASSWORD_HASH = hashPassword("zjut");
    }

    @Override
    public boolean login(String username, String password) {
        if (username.equals(CORRECT_USERNAME) && validatePassword(password)) {
            return true;
        } else {
            return false;
        }
    }

    private boolean validatePassword(String password) {
        String hashedPassword = hashPassword(password);
        return hashedPassword.equals(CORRECT_PASSWORD_HASH);
    }

    private String hashPassword(String password) {
        try {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            byte[] hashedBytes = md.digest(password.getBytes());

            StringBuilder stringBuilder = new StringBuilder();
            for (byte b : hashedBytes) {
                stringBuilder.append(String.format("%02x", b));
            }
            return stringBuilder.toString();
        } catch (NoSuchAlgorithmException e) {
            e.printStackTrace();
            return null;
        }
    }
}

//public class UserServiceBean implements UserServiceRemote {
//    public UserServiceBean() { }
//    @Override
//    public boolean login(String username, String password){
//        if(username.equals("zjut")&&password.equals("zjut")){
//            return true;
//        }else
//            return false;
//    }
//}