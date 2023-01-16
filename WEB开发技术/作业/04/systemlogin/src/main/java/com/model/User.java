package com.model;

public class User {
  private String uid;
  private String password;
  private String name;
  private String identity;

  public User() {
  }

  public User(String uid, String password, String name, String identity) {
    this.uid = uid;
    this.password = password;
    this.name = name;
    this.identity = identity;
  }

  public String getUid() {
    return this.uid;
  }

  public void setUid(String uid) {
    this.uid = uid;
  }

  public String getPassword() {
    return this.password;
  }

  public void setPassword(String password) {
    this.password = password;
  }

  public String getName() {
    return this.name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public String getIdentity() {
    return this.identity;
  }

  public void setIdentity(String identity) {
    this.identity = identity;
  }

}
