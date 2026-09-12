/*M!999999\- enable the sandbox mode */ 
-- MariaDB dump 10.19  Distrib 10.11.14-MariaDB, for debian-linux-gnu (x86_64)
--
-- Host: localhost    Database: performance_system
-- ------------------------------------------------------
-- Server version	10.11.14-MariaDB-0+deb12u2

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `assessment_score_detail`
--

DROP TABLE IF EXISTS `assessment_score_detail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `assessment_score_detail` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `assessment_id` bigint(20) NOT NULL,
  `scorer_user_id` bigint(20) NOT NULL,
  `scorer_role` varchar(64) NOT NULL,
  `raw_score` double NOT NULL,
  `comment` varchar(500) DEFAULT NULL,
  `score_time` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_score_detail_assessment` (`assessment_id`),
  KEY `fk_score_detail_user` (`scorer_user_id`),
  CONSTRAINT `fk_score_detail_assessment` FOREIGN KEY (`assessment_id`) REFERENCES `monthly_assessment` (`id`),
  CONSTRAINT `fk_score_detail_user` FOREIGN KEY (`scorer_user_id`) REFERENCES `sys_user` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=9019 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `assessment_score_detail`
--

LOCK TABLES `assessment_score_detail` WRITE;
/*!40000 ALTER TABLE `assessment_score_detail` DISABLE KEYS */;
INSERT INTO `assessment_score_detail` VALUES
(9001,7001,1,'DEPT_MANAGER',86,'部门经理评分','2026-06-15 15:36:22'),
(9002,7001,2,'TECH_DIRECTOR',90,'技术总监评分','2026-06-15 15:36:22'),
(9003,7001,4,'GENERAL_MANAGER',88,'总经理评分','2026-06-15 15:36:22'),
(9005,7002,2,'TECH_DIRECTOR',91,'技术总监评分','2026-06-15 15:36:22'),
(9006,7002,4,'GENERAL_MANAGER',94,'总经理评分','2026-06-15 15:36:22'),
(9007,7003,1,'DEPT_MANAGER',84,'部门经理评分','2026-06-15 15:36:22'),
(9008,7003,2,'TECH_DIRECTOR',87,'技术总监评分','2026-06-15 15:36:22'),
(9009,7003,4,'GENERAL_MANAGER',90,'总经理评分','2026-06-15 15:36:22'),
(9010,7006,13,'DEPT_MANAGER',80,NULL,'2026-06-16 07:28:58'),
(9011,7006,13,'TECH_DIRECTOR',80,NULL,'2026-06-16 07:29:06'),
(9012,7006,13,'GENERAL_MANAGER',80,NULL,'2026-06-16 07:29:09'),
(9013,7002,8,'DEPT_MANAGER',92.3,'测试覆盖完整，支撑及时。','2026-06-16 08:16:03'),
(9016,7007,4,'GENERAL_MANAGER',92,NULL,'2026-06-16 08:40:36'),
(9018,7007,8,'DEPT_MANAGER',55,NULL,'2026-06-16 09:14:16');
/*!40000 ALTER TABLE `assessment_score_detail` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `audit_log`
--

DROP TABLE IF EXISTS `audit_log`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `audit_log` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `operator_user_id` bigint(20) DEFAULT NULL,
  `action_type` varchar(64) NOT NULL,
  `target_type` varchar(64) NOT NULL,
  `target_id` varchar(64) NOT NULL,
  `result` varchar(32) NOT NULL,
  `action_time` timestamp NOT NULL,
  `detail` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_audit_log_operator` (`operator_user_id`),
  CONSTRAINT `fk_audit_log_operator` FOREIGN KEY (`operator_user_id`) REFERENCES `sys_user` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=8128 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `audit_log`
--

LOCK TABLES `audit_log` WRITE;
/*!40000 ALTER TABLE `audit_log` DISABLE KEYS */;
INSERT INTO `audit_log` VALUES
(8001,3,'SYSTEM_BOOTSTRAP','system','bootstrap','SUCCESS','2026-06-15 15:36:22','初始化演示数据'),
(8007,NULL,'REGISTER_USER','sys_user','9','SUCCESS','2026-06-15 17:39:42','register user testUser'),
(8010,NULL,'REGISTER_USER','sys_user','10','SUCCESS','2026-06-16 02:40:42','register user testuser01'),
(8011,NULL,'REGISTER_USER','sys_user','11','SUCCESS','2026-06-16 02:41:06','register user testUser001'),
(8020,NULL,'REGISTER_USER','sys_user','12','SUCCESS','2026-06-16 02:55:03','register user testuser03'),
(8021,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 02:55:26','login success'),
(8023,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 03:31:13','login success'),
(8048,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 03:53:55','login success'),
(8049,12,'LOGOUT','auth','12','SUCCESS','2026-06-16 06:41:43','logout success'),
(8050,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 06:55:12','login success'),
(8051,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 07:04:19','login success'),
(8052,12,'LOGOUT','auth','12','SUCCESS','2026-06-16 07:04:59','logout success'),
(8053,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 07:05:30','login success'),
(8054,NULL,'REGISTER_USER','sys_user','13','SUCCESS','2026-06-16 07:13:08','register user 超级管理员'),
(8055,12,'LOGOUT','auth','12','SUCCESS','2026-06-16 07:16:38','logout success'),
(8056,13,'LOGIN','auth','superadmin','SUCCESS','2026-06-16 07:17:13','login success'),
(8057,12,'LOGIN','auth','testuser03','SUCCESS','2026-06-16 07:17:57','login success'),
(8058,3,'UPDATE_USER','sys_user','1','SUCCESS','2026-06-16 07:18:10','update user manager01'),
(8059,3,'UPDATE_USER_STATUS','sys_user','1','SUCCESS','2026-06-16 07:18:14',NULL),
(8060,3,'UPDATE_USER_STATUS','sys_user','1','SUCCESS','2026-06-16 07:18:16',NULL),
(8061,3,'UPDATE_USER_STATUS','sys_user','6','SUCCESS','2026-06-16 07:18:22',NULL),
(8062,3,'UPDATE_USER_STATUS','sys_user','6','SUCCESS','2026-06-16 07:18:25',NULL),
(8063,3,'UPDATE_USER_STATUS','sys_user','6','SUCCESS','2026-06-16 07:18:27',NULL),
(8064,3,'UPDATE_USER_STATUS','sys_user','1','SUCCESS','2026-06-16 07:18:31',NULL),
(8065,3,'UPDATE_USER','sys_user','1','SUCCESS','2026-06-16 07:18:35','update user manager01'),
(8066,3,'UPDATE_USER_STATUS','sys_user','1','SUCCESS','2026-06-16 07:18:37',NULL),
(8067,3,'UPDATE_TASK_STATUS','weekly_task','3001','SUCCESS','2026-06-16 07:20:30','update task status'),
(8068,13,'LOGIN','auth','superadmin','SUCCESS','2026-06-16 07:20:52','login success'),
(8069,3,'UPDATE_TEAM','team','11','SUCCESS','2026-06-16 07:26:56','update team 质量保障组'),
(8070,3,'CREATE_MEMBER','team_member','1005','SUCCESS','2026-06-16 07:27:54','create member 张运维'),
(8071,3,'UPDATE_MEMBER_STATUS','team_member','1005','SUCCESS','2026-06-16 07:27:57',NULL),
(8072,3,'UPDATE_MEMBER','team_member','1005','SUCCESS','2026-06-16 07:28:05','update member 张运维'),
(8073,3,'GENERATE_ASSESSMENT','monthly_assessment','2026-06','SUCCESS','2026-06-16 07:28:23','generate assessments'),
(8074,13,'SCORE_ASSESSMENT','monthly_assessment','7006','SUCCESS','2026-06-16 07:28:58',NULL),
(8075,13,'SCORE_ASSESSMENT','monthly_assessment','7006','SUCCESS','2026-06-16 07:29:06',NULL),
(8076,13,'SCORE_ASSESSMENT','monthly_assessment','7006','SUCCESS','2026-06-16 07:29:09',NULL),
(8077,13,'FINALIZE_ASSESSMENT','monthly_assessment','7006','SUCCESS','2026-06-16 07:29:15',NULL),
(8078,3,'UPDATE_TASK','weekly_task','3002','SUCCESS','2026-06-16 07:35:38','update task 补充季度统计接口'),
(8079,3,'UPDATE_TASK_STATUS','weekly_task','3002','SUCCESS','2026-06-16 07:35:45','update task status'),
(8080,12,'LOGOUT','auth','12','SUCCESS','2026-06-16 07:36:46','logout success'),
(8081,1,'LOGIN','auth','manager01','SUCCESS','2026-06-16 07:37:04','login success'),
(8082,3,'CREATE_TASK','weekly_task','3004','SUCCESS','2026-06-16 07:42:54','create task 修复部分功能无法使用的bug'),
(8083,3,'UPDATE_USER','sys_user','2','SUCCESS','2026-06-16 07:43:48','update user director01'),
(8084,1,'LOGOUT','auth','1','SUCCESS','2026-06-16 07:55:10','logout success'),
(8085,8,'LOGIN','auth','manager02','SUCCESS','2026-06-16 07:56:40','login success'),
(8086,3,'UPDATE_USER','sys_user','9','SUCCESS','2026-06-16 07:59:53','update user testUser'),
(8087,3,'UPDATE_USER','sys_user','9','SUCCESS','2026-06-16 08:00:38','update user testUser'),
(8088,8,'LOGOUT','auth','8','SUCCESS','2026-06-16 08:02:39','logout success'),
(8089,2,'LOGIN','auth','director01','SUCCESS','2026-06-16 08:03:34','login success'),
(8090,2,'LOGOUT','auth','2','SUCCESS','2026-06-16 08:06:34','logout success'),
(8093,8,'LOGIN','auth','manager02','SUCCESS','2026-06-16 08:07:16','login success'),
(8094,8,'FINALIZE_ASSESSMENT','monthly_assessment','7002','SUCCESS','2026-06-16 08:07:33','测试覆盖完整，支撑及时。'),
(8095,3,'UPDATE_USER','sys_user','9','SUCCESS','2026-06-16 08:14:53','update user testUser'),
(8096,8,'SCORE_ASSESSMENT','monthly_assessment','7002','SUCCESS','2026-06-16 08:16:03','测试覆盖完整，支撑及时。'),
(8097,3,'GENERATE_ASSESSMENT','monthly_assessment','2026-06','SUCCESS','2026-06-16 08:16:25','generate assessments'),
(8098,8,'SCORE_ASSESSMENT','monthly_assessment','7007','SUCCESS','2026-06-16 08:16:34',NULL),
(8099,8,'SCORE_ASSESSMENT','monthly_assessment','7007','SUCCESS','2026-06-16 08:16:42',NULL),
(8100,13,'LOGOUT','auth','13','SUCCESS','2026-06-16 08:35:21','logout success'),
(8101,4,'LOGIN','auth','gm01','SUCCESS','2026-06-16 08:39:44','login success'),
(8102,4,'SCORE_ASSESSMENT','monthly_assessment','7007','SUCCESS','2026-06-16 08:40:36',NULL),
(8103,4,'LOGOUT','auth','4','SUCCESS','2026-06-16 08:40:57','logout success'),
(8104,13,'LOGIN','auth','superadmin','SUCCESS','2026-06-16 08:41:10','login success'),
(8105,13,'FINALIZE_ASSESSMENT','monthly_assessment','7002','SUCCESS','2026-06-16 08:41:37','测试覆盖完整，支撑及时。'),
(8106,13,'LOGOUT','auth','13','SUCCESS','2026-06-16 08:42:51','logout success'),
(8108,1,'LOGIN','auth','manager01','SUCCESS','2026-06-16 08:43:10','login success'),
(8109,1,'LOGOUT','auth','1','SUCCESS','2026-06-16 09:00:47','logout success'),
(8111,13,'LOGIN','auth','superadmin','SUCCESS','2026-06-16 09:01:02','login success'),
(8112,13,'LOGOUT','auth','13','SUCCESS','2026-06-16 09:13:36','logout success'),
(8113,8,'LOGIN','auth','manager02','SUCCESS','2026-06-16 09:13:46','login success'),
(8114,8,'SCORE_ASSESSMENT','monthly_assessment','7007','SUCCESS','2026-06-16 09:14:08',NULL),
(8115,8,'SCORE_ASSESSMENT','monthly_assessment','7007','SUCCESS','2026-06-16 09:14:16',NULL),
(8116,8,'LOGOUT','auth','8','SUCCESS','2026-06-16 09:14:33','logout success'),
(8117,13,'LOGIN','auth','superadmin','SUCCESS','2026-06-16 09:14:42','login success'),
(8118,3,'LOGIN','auth','admin01','SUCCESS','2026-06-30 10:54:19','login success'),
(8119,3,'LOGOUT','auth','3','SUCCESS','2026-06-30 12:24:40','logout success'),
(8121,2,'LOGIN','auth','director01','SUCCESS','2026-06-30 12:25:01','login success'),
(8122,2,'LOGOUT','auth','2','SUCCESS','2026-06-30 13:31:55','logout success'),
(8123,3,'LOGIN','auth','admin01','SUCCESS','2026-06-30 13:32:04','login success'),
(8124,3,'LOGOUT','auth','3','SUCCESS','2026-06-30 13:38:04','logout success'),
(8127,2,'LOGIN','auth','director01','SUCCESS','2026-06-30 13:38:34','login success');
/*!40000 ALTER TABLE `audit_log` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `monthly_assessment`
--

DROP TABLE IF EXISTS `monthly_assessment`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `monthly_assessment` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `member_id` bigint(20) NOT NULL,
  `assess_month` varchar(16) NOT NULL,
  `dept_manager_score` double DEFAULT NULL,
  `tech_director_score` double DEFAULT NULL,
  `general_manager_score` double DEFAULT NULL,
  `final_score` double DEFAULT NULL,
  `rating_level` varchar(8) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `comment` varchar(500) DEFAULT NULL,
  `created_at` timestamp NOT NULL,
  `updated_at` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_monthly_assessment_member` (`member_id`),
  CONSTRAINT `fk_monthly_assessment_member` FOREIGN KEY (`member_id`) REFERENCES `team_member` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=7008 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `monthly_assessment`
--

LOCK TABLES `monthly_assessment` WRITE;
/*!40000 ALTER TABLE `monthly_assessment` DISABLE KEYS */;
INSERT INTO `monthly_assessment` VALUES
(7001,1001,'2026-05',86,90,88,87.8,'B','FINALIZED','交付稳定，接口质量较高。','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(7002,1002,'2026-05',92.3,91,94,92.42,'A','FINALIZED','测试覆盖完整，支撑及时。','2026-06-15 15:36:22','2026-06-16 08:41:37'),
(7003,1003,'2026-06',84,87,90,86.7,'B','FINALIZED','本月已完成评分归档。','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(7004,1004,'2026-06',NULL,NULL,NULL,NULL,NULL,'DRAFT',NULL,'2026-06-15 15:36:22','2026-06-15 15:36:22'),
(7006,1005,'2026-06',80,80,80,80,'B','FINALIZED',NULL,'2026-06-16 07:28:23','2026-06-16 07:29:15'),
(7007,1002,'2026-06',55,NULL,92,NULL,NULL,'SCORING',NULL,'2026-06-16 08:16:25','2026-06-16 09:14:16');
/*!40000 ALTER TABLE `monthly_assessment` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `project`
--

DROP TABLE IF EXISTS `project`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `project` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `project_name` varchar(120) NOT NULL,
  `owner_member_id` bigint(20) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `priority` varchar(32) NOT NULL,
  `start_date` date NOT NULL,
  `end_date` date DEFAULT NULL,
  `milestone` varchar(255) DEFAULT NULL,
  `description` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_project_owner` (`owner_member_id`),
  CONSTRAINT `fk_project_owner` FOREIGN KEY (`owner_member_id`) REFERENCES `team_member` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=2003 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `project`
--

LOCK TABLES `project` WRITE;
/*!40000 ALTER TABLE `project` DISABLE KEYS */;
INSERT INTO `project` VALUES
(2001,'绩效考核系统 V1.0',1001,'IN_PROGRESS','HIGH','2026-06-15','2026-06-15','完成 MVP 闭环','课程设计主项目'),
(2002,'测试平台升级',1002,'PLANNING','MEDIUM','2026-06-15','2026-06-15','完善自动化测试链路','支撑质量保障组日常测试工作');
/*!40000 ALTER TABLE `project` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `sys_role`
--

DROP TABLE IF EXISTS `sys_role`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `sys_role` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `role_code` varchar(64) NOT NULL,
  `role_name` varchar(64) NOT NULL,
  `description` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `role_code` (`role_code`)
) ENGINE=InnoDB AUTO_INCREMENT=14 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sys_role`
--

LOCK TABLES `sys_role` WRITE;
/*!40000 ALTER TABLE `sys_role` DISABLE KEYS */;
INSERT INTO `sys_role` VALUES
(1,'SYSTEM_ADMIN','系统管理员','负责用户、角色、团队、成员和系统基础数据维护'),
(2,'DEPT_MANAGER','部门经理','负责团队任务分配、周报审核和月度评分'),
(3,'TECH_DIRECTOR','技术总监','负责项目监督、技术评分和统计查看'),
(4,'GENERAL_MANAGER','总经理','负责综合评分与最终归档'),
(5,'AUDIT_ADMIN','审计管理员','负责审计日志与安全检查'),
(6,'TEAM_MEMBER','团队成员','负责任务执行、进度更新和周报提交');
/*!40000 ALTER TABLE `sys_role` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `sys_user`
--

DROP TABLE IF EXISTS `sys_user`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `sys_user` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `username` varchar(64) NOT NULL,
  `real_name` varchar(64) NOT NULL,
  `password_sm3` varchar(128) NOT NULL,
  `mobile` varchar(32) DEFAULT NULL,
  `email` varchar(128) DEFAULT NULL,
  `team_id` bigint(20) DEFAULT NULL,
  `member_id` bigint(20) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `must_change_password` tinyint(1) NOT NULL DEFAULT 1,
  `password_expire_at` timestamp NULL DEFAULT NULL,
  `last_login_at` timestamp NULL DEFAULT NULL,
  `created_at` timestamp NOT NULL,
  `updated_at` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `username` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=14 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sys_user`
--

LOCK TABLES `sys_user` WRITE;
/*!40000 ALTER TABLE `sys_user` DISABLE KEYS */;
INSERT INTO `sys_user` VALUES
(1,'manager01','张经理','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4',NULL,'[EMAIL]',10,1001,'ENABLED',0,'2026-06-15 15:36:22','2026-06-16 08:43:10','2026-06-15 15:36:22','2026-06-16 08:43:10'),
(2,'director01','李总监','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4',NULL,'[EMAIL]',10,NULL,'ENABLED',0,'2026-06-15 15:36:22','2026-06-30 13:38:34','2026-06-15 15:36:22','2026-06-30 13:38:34'),
(3,'admin01','系统管理员','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-06-15 15:36:22','2026-06-30 13:32:04','2026-06-15 15:36:22','2026-06-30 13:32:04'),
(4,'gm01','王总经理','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-06-15 15:36:22','2026-06-16 08:39:44','2026-06-15 15:36:22','2026-06-16 08:39:44'),
(5,'audit01','审计管理员','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(6,'dev01','王开发','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',10,1001,'ENABLED',1,'2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-16 07:18:27'),
(7,'qa01','赵测试','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',11,1002,'ENABLED',0,'2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(8,'manager02','周经理','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',11,NULL,'ENABLED',0,'2026-06-15 15:36:22','2026-06-16 09:13:46','2026-06-15 15:36:22','2026-06-16 09:13:46'),
(9,'testUser','测试用户01','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4',NULL,'[EMAIL]',NULL,NULL,'ENABLED',0,'2026-09-13 17:39:42',NULL,'2026-06-15 17:39:42','2026-06-16 08:14:53'),
(10,'testuser01','测试用户01','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-09-14 02:40:42',NULL,'2026-06-16 02:40:42','2026-06-16 02:40:42'),
(11,'testUser001','testUser001','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-09-14 02:41:06',NULL,'2026-06-16 02:41:06','2026-06-16 02:41:06'),
(12,'testuser03','测试用户01','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-09-14 02:55:03','2026-06-16 07:17:57','2026-06-16 02:55:03','2026-06-16 07:17:57'),
(13,'superadmin','折乙','f96eef7d8b40c8ec962908a95218b51ce5082d86012e753c3a63bd2d2b37f5a4','[PHONE]','[EMAIL]',NULL,NULL,'ENABLED',0,'2026-09-14 07:13:08','2026-06-16 09:14:42','2026-06-16 07:13:08','2026-06-16 09:14:42');
/*!40000 ALTER TABLE `sys_user` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `sys_user_role`
--

DROP TABLE IF EXISTS `sys_user_role`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `sys_user_role` (
  `user_id` bigint(20) NOT NULL,
  `role_id` bigint(20) NOT NULL,
  PRIMARY KEY (`user_id`,`role_id`),
  KEY `fk_sys_user_role_role` (`role_id`),
  CONSTRAINT `fk_sys_user_role_role` FOREIGN KEY (`role_id`) REFERENCES `sys_role` (`id`),
  CONSTRAINT `fk_sys_user_role_user` FOREIGN KEY (`user_id`) REFERENCES `sys_user` (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sys_user_role`
--

LOCK TABLES `sys_user_role` WRITE;
/*!40000 ALTER TABLE `sys_user_role` DISABLE KEYS */;
INSERT INTO `sys_user_role` VALUES
(1,2),
(2,3),
(3,1),
(4,4),
(5,5),
(6,6),
(7,6),
(8,2),
(9,6),
(10,6),
(11,6),
(12,6),
(13,1),
(13,2),
(13,3),
(13,4),
(13,5),
(13,6);
/*!40000 ALTER TABLE `sys_user_role` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_progress`
--

DROP TABLE IF EXISTS `task_progress`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `task_progress` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `task_id` bigint(20) NOT NULL,
  `progress_rate` int(11) NOT NULL,
  `issue_desc` varchar(500) DEFAULT NULL,
  `comment` varchar(500) NOT NULL,
  `update_time` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_task_progress_task` (`task_id`),
  CONSTRAINT `fk_task_progress_task` FOREIGN KEY (`task_id`) REFERENCES `weekly_task` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=4003 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_progress`
--

LOCK TABLES `task_progress` WRITE;
/*!40000 ALTER TABLE `task_progress` DISABLE KEYS */;
INSERT INTO `task_progress` VALUES
(4001,3001,60,NULL,'完成接口联调并修复字段映射','2026-06-15 15:36:22'),
(4002,3003,35,'联调环境接口超时','已完成测试清单，等待环境恢复','2026-06-15 15:36:22');
/*!40000 ALTER TABLE `task_progress` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `team`
--

DROP TABLE IF EXISTS `team`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `team` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `team_name` varchar(100) NOT NULL,
  `department_name` varchar(100) NOT NULL,
  `manager_user_id` bigint(20) DEFAULT NULL,
  `description` varchar(255) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `created_at` timestamp NOT NULL,
  `updated_at` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_team_manager` (`manager_user_id`),
  CONSTRAINT `fk_team_manager` FOREIGN KEY (`manager_user_id`) REFERENCES `sys_user` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=12 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `team`
--

LOCK TABLES `team` WRITE;
/*!40000 ALTER TABLE `team` DISABLE KEYS */;
INSERT INTO `team` VALUES
(10,'平台研发组','研发中心',1,'负责平台研发和绩效模块实现','ACTIVE','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(11,'质量保障组','研发中心',8,'负责测试、质量与验收支持','ACTIVE','2026-06-15 15:36:22','2026-06-16 07:26:56');
/*!40000 ALTER TABLE `team` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `team_member`
--

DROP TABLE IF EXISTS `team_member`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `team_member` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `name` varchar(64) NOT NULL,
  `team_id` bigint(20) DEFAULT NULL,
  `position` varchar(100) DEFAULT NULL,
  `job_level` varchar(32) DEFAULT NULL,
  `phone` varchar(32) DEFAULT NULL,
  `email` varchar(128) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `entry_date` date DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_team_member_team` (`team_id`),
  CONSTRAINT `fk_team_member_team` FOREIGN KEY (`team_id`) REFERENCES `team` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=1006 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `team_member`
--

LOCK TABLES `team_member` WRITE;
/*!40000 ALTER TABLE `team_member` DISABLE KEYS */;
INSERT INTO `team_member` VALUES
(1001,'王开发',10,'Java开发工程师','P5','[PHONE]','[EMAIL]','ACTIVE','2026-06-15'),
(1002,'赵测试',11,'测试工程师','P4','[PHONE]','[EMAIL]','ACTIVE','2026-06-15'),
(1003,'钱后端',10,'后端工程师','P5','[PHONE]','[EMAIL]','ACTIVE','2026-06-15'),
(1004,'孙前端',10,'前端工程师','P5','[PHONE]','[EMAIL]','ONBOARDING','2026-06-15'),
(1005,'张运维',10,'平台运维','P5',NULL,NULL,'ACTIVE','2026-06-16');
/*!40000 ALTER TABLE `team_member` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `weekly_report`
--

DROP TABLE IF EXISTS `weekly_report`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `weekly_report` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `member_id` bigint(20) NOT NULL,
  `week_no` varchar(32) NOT NULL,
  `summary` varchar(1000) NOT NULL,
  `next_plan` varchar(1000) NOT NULL,
  `manager_comment` varchar(1000) DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `created_at` timestamp NOT NULL,
  `updated_at` timestamp NOT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_weekly_report_member` (`member_id`),
  CONSTRAINT `fk_weekly_report_member` FOREIGN KEY (`member_id`) REFERENCES `team_member` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=5003 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `weekly_report`
--

LOCK TABLES `weekly_report` WRITE;
/*!40000 ALTER TABLE `weekly_report` DISABLE KEYS */;
INSERT INTO `weekly_report` VALUES
(5001,1001,'2026-W24','本周完成考核模块接口联调和图表接口设计。','补充季度汇总与日志检索。','内容完整，可作为月度评分依据。','REVIEWED','2026-06-15 15:36:22','2026-06-15 15:36:22'),
(5002,1002,'2026-W24','本周整理测试计划并跟进联调问题。','完成测试回归与问题关闭。',NULL,'SUBMITTED','2026-06-15 15:36:22','2026-06-15 15:36:22');
/*!40000 ALTER TABLE `weekly_report` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `weekly_task`
--

DROP TABLE IF EXISTS `weekly_task`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8mb4 */;
CREATE TABLE `weekly_task` (
  `id` bigint(20) NOT NULL AUTO_INCREMENT,
  `title` varchar(200) NOT NULL,
  `project_id` bigint(20) DEFAULT NULL,
  `assignee_member_id` bigint(20) NOT NULL,
  `priority` varchar(32) NOT NULL,
  `deadline` date DEFAULT NULL,
  `status` varchar(32) NOT NULL,
  `progress_rate` int(11) NOT NULL DEFAULT 0,
  `week_no` varchar(32) NOT NULL,
  `plan_start_date` date DEFAULT NULL,
  `plan_end_date` date DEFAULT NULL,
  `issue_desc` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`),
  KEY `fk_weekly_task_project` (`project_id`),
  KEY `fk_weekly_task_assignee` (`assignee_member_id`),
  CONSTRAINT `fk_weekly_task_assignee` FOREIGN KEY (`assignee_member_id`) REFERENCES `team_member` (`id`),
  CONSTRAINT `fk_weekly_task_project` FOREIGN KEY (`project_id`) REFERENCES `project` (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3005 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_unicode_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `weekly_task`
--

LOCK TABLES `weekly_task` WRITE;
/*!40000 ALTER TABLE `weekly_task` DISABLE KEYS */;
INSERT INTO `weekly_task` VALUES
(3001,'完成月度考核接口联调',2001,1001,'HIGH','2026-06-15','IN_PROGRESS',67,'2026-W24','2026-06-15','2026-06-15',NULL),
(3002,'补充季度统计接口',2001,1003,'MEDIUM','2026-06-15','TODO',20,'2026-W24','2026-06-15','2026-06-15',NULL),
(3003,'整理测试用例与回归报告',2002,1002,'MEDIUM','2026-06-15','BLOCKED',35,'2026-W24','2026-06-15','2026-06-15','联调环境不稳定'),
(3004,'修复部分功能无法使用的bug',2001,1001,'MEDIUM',NULL,'TODO',0,'2026-W26',NULL,NULL,NULL);
/*!40000 ALTER TABLE `weekly_task` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2026-06-30 22:35:18
