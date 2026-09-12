package com.campus.demo.service;

import com.campus.demo.dto.CreateTeamRequest;
import com.campus.demo.entity.Team;

import java.util.List;

public interface TeamService {

    Team getById(Long id);

    List<Team> listTeams();

    Team createTeam(CreateTeamRequest request);
}
