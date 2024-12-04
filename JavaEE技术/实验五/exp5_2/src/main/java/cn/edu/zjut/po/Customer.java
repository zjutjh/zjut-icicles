package cn.edu.zjut.po;

import java.util.Date;

public class Customer implements java.io.Serializable
{
    private int customerId;
    private String account;
    private String password;
    private String name;
    private Boolean sex;
    private Date birthday;
    private ContactInfo contactInfo;

    public Customer(int customerId)
    {
        this.customerId = customerId;
    }

    public Customer()
    {
    }

    public Customer(int customerId, String account, String password, String name, Boolean sex, Date birthday, ContactInfo contactInfo)
    {
        this.customerId = customerId;
        this.account = account;
        this.password = password;
        this.name = name;
        this.sex = sex;
        this.birthday = birthday;
        this.contactInfo = contactInfo;
    }

    public int getCustomerId()
    {
        return customerId;
    }

    public void setCustomerId(int customerId)
    {
        this.customerId = customerId;
    }

    public String getAccount()
    {
        return account;
    }

    public void setAccount(String account)
    {
        this.account = account;
    }

    public String getPassword()
    {
        return password;
    }

    public void setPassword(String password)
    {
        this.password = password;
    }

    public String getName()
    {
        return name;
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public Boolean getSex()
    {
        return sex;
    }

    public void setSex(Boolean sex)
    {
        this.sex = sex;
    }

    public Date getBirthday()
    {
        return birthday;
    }

    public void setBirthday(Date birthday)
    {
        this.birthday = birthday;
    }

    public ContactInfo getContactInfo()
    {
        return contactInfo;
    }

    public void setContactInfo(ContactInfo contactInfo)
    {
        this.contactInfo = contactInfo;
    }
}
