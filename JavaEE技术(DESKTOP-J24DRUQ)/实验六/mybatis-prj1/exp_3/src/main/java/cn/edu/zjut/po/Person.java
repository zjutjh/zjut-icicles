package cn.edu.zjut.po;

public class Person
{
    private int id;
    private String name;
    private int age;
    private IDcard idcard;

    public Person()
    {
    }

    public Person(int id, String name, int age, IDcard idcard)
    {
        this.id = id;
        this.name = name;
        this.age = age;
        this.idcard = idcard;
    }

    public int getId()
    {
        return id;
    }

    public void setId(int id)
    {
        this.id = id;
    }

    public String getName()
    {
        return name;
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public int getAge()
    {
        return age;
    }

    public void setAge(int age)
    {
        this.age = age;
    }

    public IDcard getIdcard()
    {
        return idcard;
    }

    public void setIdcard(IDcard idcard)
    {
        this.idcard = idcard;
    }

    @Override
    public String toString()
    {
        return "Person{" + "id=" + id + ", name='" + name + '\'' + ", age=" + age + ", idcard_id=" + idcard + '}';
    }
}