package cn.edu.zjut.pojo;

public class SelectPersonById
{
    private Integer id;
    private String name;
    private Integer age;
    private String code;

    //省略 setter 和 getter 方法
    @Override
    public String toString()
    {
        return "Person [id=" + id + ",name=" + name + ", age = " + age + ", code = " + code + "]";
    }
}