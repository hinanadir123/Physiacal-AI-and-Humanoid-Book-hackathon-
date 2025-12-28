import React, { useState, useEffect } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useLocation } from '@docusaurus/router';
import { isAuthenticated, getAuthToken } from '../../utils/auth';
import PersonalizationButton from '../../components/Auth/PersonalizationButton';

const Navbar = (props) => {
  const [isAuth, setIsAuth] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const checkAuth = () => {
      const authStatus = isAuthenticated();
      setIsAuth(authStatus);
    };

    checkAuth();

    // Listen for storage events to update auth status across tabs
    const handleStorageChange = () => {
      checkAuth();
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  return (
    <>
      <OriginalNavbar {...props} />
      {/* Personalization button only shows when authenticated and not on auth pages */}
      {isAuth && !location.pathname.includes('/auth/') && (
        <div style={{ position: 'fixed', right: '10px', top: '10px', zIndex: 1000 }}>
          <PersonalizationButton />
        </div>
      )}
    </>
  );
};

export default Navbar;