package com.campus.demo;

import org.mybatis.spring.annotation.MapperScan;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
@MapperScan("com.campus.demo.mapper")
public class PerformanceApplication {

    public static void main(String[] args) {
        SpringApplication.run(PerformanceApplication.class, args);
    }
}
