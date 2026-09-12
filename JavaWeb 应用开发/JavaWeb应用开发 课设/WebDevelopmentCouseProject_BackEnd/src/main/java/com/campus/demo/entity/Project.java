package com.campus.demo.entity;

import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.ProjectStatus;
import lombok.Data;

import java.time.LocalDate;

@Data
public class Project {

    private Long id;
    private String projectName;
    private Long ownerId;
    private String ownerName;
    private ProjectStatus status;
    private PriorityLevel priority;
    private LocalDate startDate;
    private LocalDate endDate;
    private String milestone;
    private String description;
}
