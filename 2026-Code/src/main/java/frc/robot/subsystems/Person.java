package frc.robot.subsystems;

import java.util.Array;

public class Person {
    private String m_name;
    private String[] m_nicknames;
    private int m_age;
    private boolean m_isEmployed;
    private double m_height;

    public Person(String p_name, String[] p_nicknames, int p_age, double p_height, boolean p_isEmployed) {
        m_name = p_name;
        m_nicknames = p_nicknames;
        p_age = m_age;
        m_isEmployed = p_isEmployed;
        m_height = p_height;
    }

    public String getName() {
        return m_name;
    }

    public String[] getNickname() {
        return m_nicknames;
    }

    public int getAge() {
        return m_age;
    }

    public double getHeight() {
        return m_height;
    }

    public boolean employmentStatus() {
        return m_isEmployed;
    }

    public static void main(String[] args) {
        Person Siwon = new Person("Siwon", {"Seoul Sister", "Triple S"}, 15, 8.2, false);
    }
}
