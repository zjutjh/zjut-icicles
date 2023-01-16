- 默认构造函数
- 含参构造函数
- `getter` & `setter`

> 12/13 上课老师说 getter & setter 可以不写？

## template

```java
package com.model;

public class Customer {
  private String email;
  private String custName;

  public Customer() {
  }

  public Customer(String email, String password, String custName, String phone) {
    this.email = email;
    this.custName = custName;
  }

  public String getEmail() {
    return this.email;
  }

  public String getCustName() {
    return this.custName;
  }

  public void setEmail(String email) {
    this.email = email;
  }

  public void setCustName(String custName) {
    this.custName = custName;
  }
}
```