import java.util.*;

import com.alibaba.fastjson.JSON;
import com.alibaba.fastjson.annotation.JSONField;

class Person {

  @JSONField(name = "AGE")
  private int age;

  @JSONField(name = "FULL NAME")
  private String fullName;

  @JSONField(name = "DATE OF BIRTH")
  private Date dateOfBirth;

  public Person(int age, String fullName, Date dateOfBirth) {
    super();
    this.age = age;
    this.fullName = fullName;
    this.dateOfBirth = dateOfBirth;
  }

  // 标准 getters & setters
}

public class App {
  private List<Person> listOfPersons = new ArrayList<Person>();

  public void setUp() {
    listOfPersons.add(new Person(15, "John Doe", new Date()));
    listOfPersons.add(new Person(20, "Janette Doe", new Date()));
  }

  public void whenJavaList_thanConvertToJsonCorrect() {
    String jsonOutput = JSON.toJSONString(listOfPersons);
    System.out.println(jsonOutput);
  }

  public static void main(String[] args) {
    App a = new App();
    a.whenJavaList_thanConvertToJsonCorrect();
  }
}
