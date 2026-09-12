package com.campus.demo.controller;

import com.campus.demo.common.Result;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.LinkedHashMap;
import java.util.Map;

@RestController
public class HomeController {

    @GetMapping("/")
    public Result<Map<String, Object>> home() {
        Map<String, Object> data = new LinkedHashMap<>();
        data.put("project", "software-team-performance-backend");
        data.put("message", "项目已启动，可以开始联调接口");
        data.put("home", "http://localhost:8081/");
        data.put("login", "http://localhost:8081/api/auth/login");
        data.put("users", "http://localhost:8081/api/users");
        data.put("teams", "http://localhost:8081/api/teams");
        data.put("h2Console", "http://localhost:8081/h2-console");
        return Result.ok(data);
    }
}
