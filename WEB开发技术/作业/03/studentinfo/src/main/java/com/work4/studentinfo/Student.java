package com.work4.studentinfo;

public class Student {

  private String stuid;
  private String name;
  private String major;

  public Student() {
    this.stuid = "";
    this.name = "";
    this.major = "";
  }

  public String getName() {
    return this.name;
  }

  public String getStuid() {
    return this.stuid;
  }

  public String getMajor() {
    return this.major;
  }

  public void setMajor(String _major) {
    this.major = _major;
  }

  public void setName(String _name) {
    this.name = _name;
  }

  public void setStuid(String _stuid) {
    this.stuid = _stuid;
  }

  @Override
  public String toString() {
    return this.stuid + " " + this.name + " " + this.major;
  }

}