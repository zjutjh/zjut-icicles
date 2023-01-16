package com.model;

public class Customer {
  private String email;
  private String password;
  private String custName;
  private String phone;

  public Customer() {
  }

  public Customer(String email, String password, String custName, String phone) {
    this.email = email;
    this.password = password;
    this.custName = custName;
    this.phone = phone;
  }

  @Override
  public String toString() {
    return this.email + " " + this.password + " " + this.custName + " " + this.phone;
  }

  public String getEmail() {
    return this.email;
  }

  public String getPassword() {
    return this.password;
  }

  public String getCustName() {
    return this.custName;
  }

  public String getPhone() {
    return this.phone;
  }

  public void setEmail(String email) {
    this.email = email;
  }

  public void setPassword(String password) {
    this.password = password;
  }

  public void setCustName(String custName) {
    this.custName = custName;
  }

  public void setPhone(String phone) {
    this.phone = phone;
  }
}