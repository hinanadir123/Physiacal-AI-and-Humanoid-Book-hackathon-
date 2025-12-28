import React, { useState, useEffect } from 'react';
import { authAPI } from '../../utils/auth';
import '../../css/auth.css';

const ProfilePage = () => {
  const [profile, setProfile] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    fetchProfile();
  }, []);

  const fetchProfile = async () => {
    try {
      const response = await authAPI.getCurrentProfile();
      setProfile(response.data);
    } catch (err) {
      setError('Failed to load profile data');
      console.error('Error fetching profile:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return <div className="loading">Loading profile...</div>;
  }

  if (error) {
    return <div className="error-message">{error}</div>;
  }

  if (!profile) {
    return <div className="error-message">No profile data found</div>;
  }

  return (
    <div className="profile-page">
      <div className="container">
        <h1>Profile</h1>
        <div className="profile-info">
          <h2>Personal Information</h2>
          <p><strong>Name:</strong> {profile.first_name} {profile.last_name}</p>
          <p><strong>Experience Level:</strong> {profile.experience_level || 'Not specified'}</p>
        </div>

        {profile.software_skills && (
          <div className="profile-section">
            <h2>Software Skills</h2>
            {profile.software_skills.programming_languages && profile.software_skills.programming_languages.length > 0 && (
              <div>
                <h3>Programming Languages</h3>
                <ul>
                  {profile.software_skills.programming_languages.map((lang, index) => (
                    <li key={index}>{lang.name} (Proficiency: {lang.proficiency})</li>
                  ))}
                </ul>
              </div>
            )}
            {profile.software_skills.frameworks && profile.software_skills.frameworks.length > 0 && (
              <div>
                <h3>Frameworks</h3>
                <ul>
                  {profile.software_skills.frameworks.map((framework, index) => (
                    <li key={index}>{framework.name} (Proficiency: {framework.proficiency})</li>
                  ))}
                </ul>
              </div>
            )}
            {profile.software_skills.tools && profile.software_skills.tools.length > 0 && (
              <div>
                <h3>Tools</h3>
                <ul>
                  {profile.software_skills.tools.map((tool, index) => (
                    <li key={index}>{tool.name} (Proficiency: {tool.proficiency})</li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        )}

        {profile.hardware_access && (
          <div className="profile-section">
            <h2>Hardware Access</h2>
            {profile.hardware_access.development_kits && profile.hardware_access.development_kits.length > 0 && (
              <div>
                <h3>Development Kits</h3>
                <ul>
                  {profile.hardware_access.development_kits.map((kit, index) => (
                    <li key={index}>{kit}</li>
                  ))}
                </ul>
              </div>
            )}
            {profile.hardware_access.robotics_platforms && profile.hardware_access.robotics_platforms.length > 0 && (
              <div>
                <h3>Robotics Platforms</h3>
                <ul>
                  {profile.hardware_access.robotics_platforms.map((platform, index) => (
                    <li key={index}>{platform}</li>
                  ))}
                </ul>
              </div>
            )}
            {profile.hardware_access.simulation_environments && profile.hardware_access.simulation_environments.length > 0 && (
              <div>
                <h3>Simulation Environments</h3>
                <ul>
                  {profile.hardware_access.simulation_environments.map((env, index) => (
                    <li key={index}>{env}</li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default ProfilePage;