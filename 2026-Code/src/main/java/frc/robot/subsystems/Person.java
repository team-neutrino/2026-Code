package frc.robot.subsystems;

import java.util.List;

public class Person {
    private String m_name;
    private List<String> m_nicknames;
    private int m_age;
    private boolean m_isEmployed;
    private double m_height;

    public Person(String p_name, List<String> p_nicknames, int p_age, double p_height, boolean p_isEmployed) {
        m_name = p_name;
        m_nicknames = p_nicknames;
        m_age = p_age;
        m_isEmployed = p_isEmployed;
        m_height = p_height;
    }

    public String getName() {
        return m_name;
    }

    public List<String> getNickname() {
        return m_nicknames;
    }

    public int getAge() {
        return m_age;
    }

    public double getHeight() {
        return m_height;
    }

    public boolean isEmployed() {
        return m_isEmployed;
    }

    public static void main(String[] args) {
        Person Siwon = new Person("Siwon", List.of("Seoul Sister", "Triple S"), 15, 8.2, false);
        System.out.println("Name: " + Siwon.getName() + ", nicknames: " + Siwon.getNickname() + ", age: " + Siwon.getAge() + ", height: " + Siwon.getHeight() + " ft" + ", employed: " + Siwon.isEmployed());
    }
}
