package cn.edu.zjut.model;

public class UserBean
{
    // 属性声明
    private String username;
    private String password;
    private String type; // 新增用户类型属性

    //构造方法
    public UserBean()
    {
    }

    // get 方法
    public String getUsername()
    {
        return username;
    }

    public String getPassword()
    {
        return password;
    }

    public String getType() {
        return type;
    }

    // set 方法
    public void setUsername(String username)
    {
        this.username = username;
    }

    public void setPassword(String password)
    {
        this.password = password;
    }

    public void setType(String type) {
        this.type = type;
    }
}
