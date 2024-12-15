package cn.edu.zjut.po;

public class MyUser
{
    private int uid;
    private String uname;
    private String usex;

    // 无参构造函数
    public MyUser()
    {
    }

    // 带参构造函数
    public MyUser(int uid, String uname, String usex)
    {
        this.uid = uid;
        this.uname = uname;
        this.usex = usex;
    }

    // Getter方法
    public int getUid()
    {
        return uid;
    }

    // Setter方法
    public void setUid(int uid)
    {
        this.uid = uid;
    }

    public String getUname()
    {
        return uname;
    }

    public void setUname(String uname)
    {
        this.uname = uname;
    }

    public String getUsex()
    {
        return usex;
    }

    public void setUsex(String usex)
    {
        this.usex = usex;
    }
}
